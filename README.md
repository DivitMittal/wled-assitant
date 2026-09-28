# wled-assitant

Firmware for the desk ambient light: an ESP32-S3 SuperMini (ESP32-S3FH4R2) running **stock upstream WLED v16.0.1** and one self-contained usermod. It drives 144 WS2812B LEDs through a TXS0108E level shifter, takes HyperHDR frames as **Adalight over the board's own USB-C port**, and reads an HTU21D sensor.

> Status: the firmware compiles and links, and the logic was checked against the WLED/arduino-esp32 source. **It has not yet been run on hardware.** Work through the [first-power sequence](#first-power--test-sequence) before relying on it.

## Layout

```
wled-assistant/                     (project name: wled-assitant)
├── platformio_override.ini         build env + the ONE place pins/LED count/current limit are set
├── usermods/wled_assitant/         custom usermod (library.json + wled_assitant.cpp)
├── scripts/bootstrap.sh            clones WLED v16.0.1 into wled/, refuses a modified core, links the override
├── scripts/build.sh                bootstrap + web UI + firmware build (`build.sh upload` to flash)
├── tools/adalight_test.py          sends Adalight frames over USB for testing without HyperHDR
└── wled/                           upstream checkout (gitignored, never edited)
```

**Upstream WLED files modified: none.** `bootstrap.sh` checks this every time it runs (line-ending-only differences in upstream's CRLF files are ignored). The only file added inside `wled/` is the `platformio_override.ini` symlink, which is WLED's supported override mechanism.

## Architecture

```
Host ──USB-C (USB-Serial/JTAG, CDC-ACM)──▶ ESP32-S3
  HyperHDR "Adalight/Ada"                   │
                                            ├─ Serial (= HWCDC) ─▶ stock wled_serial.cpp Adalight parser
                                            │                       └▶ realtimeLock() → WLED LED engine (RMT)
                                            │                           → GPIO6 → TXS0108E A7/B7 → 220R → DIN
                                            ├─ usermod wled_assitant
                                            │    ├ OE (GPIO7): LOW from static init; HIGH once LED bus is idle
                                            │    ├ HTU21D on WLED's global I2C (SDA 9 / SCL 8), non-blocking
                                            │    ├ MQTT + HA discovery (only when WLED's MQTT is configured)
                                            │    └ /json/info diagnostics
                                            └─ Wi-Fi: Web UI, JSON API, HA WLED integration, OTA, (MQTT)
```

### USB transport: how stock Adalight reaches the USB port

Traced in the v16.0.1 source:

1. `WLED::loop()` calls `handleSerial()` (`wled00/wled_serial.cpp`) on every pass when `WLED_ENABLE_ADALIGHT` is defined, which is the default. That function parses Adalight (`Ada` + count + checksum + RGB) from **`Serial`**, then calls `setRealtimePixel()`, `realtimeLock(realtimeTimeoutMs, REALTIME_MODE_ADALIGHT)` and `strip.show()`.
2. With `ARDUINO_USB_MODE=1` and `ARDUINO_USB_CDC_ON_BOOT=1` (the `lolin_s3_mini` board JSON sets the first flag and the override sets both), arduino-esp32 2.0.18 defines `HWCDC Serial`. That is the S3's built-in USB-Serial/JTAG peripheral on the USB-C connector, so stock WLED already reads Adalight from USB with no core patch.
3. `handleSerial()` only runs when `serialCanRX` is true, which means GPIO44 (UART0 RX) is not allocated. The usermod reports this in Info, and the build rejects pins 43/44.
4. **Gap found and closed from the usermod.** HWCDC's RX queue defaults to 256 bytes, and its ISR **silently drops** incoming bytes when the queue is full (`hw_cdc_isr_handler` in `HWCDC.cpp`), with no USB flow control. One 144-LED frame is 438 bytes, so even a short WLED loop stall would corrupt frames. The usermod constructor calls `Serial.setRxBufferSize(2048)` during C++ static initialisation, before WLED's `Serial.begin()`. `HWCDC::begin()` keeps a queue that is already allocated, so it is never swapped while the USB ISR is running. 2048 bytes holds about 4.7 frames, roughly 78 ms of stall tolerance at 60 fps.

Why USB-Serial/JTAG (`USB_MODE=1`) and not TinyUSB/OTG (`USB_MODE=0`): the port is implemented in hardware, so it survives firmware crashes, and esptool can auto-reset and flash through it. The board is always recoverable on the same cable.

**Throughput:** a frame is 6 header bytes plus 144 × 3 data bytes = 438 bytes. At 60 fps that is 26.3 kB/s (≈263 kbaud-equivalent at 10 bits per byte). USB full-speed bulk handles several hundred kB/s or more, so 60 fps uses only a few percent of it. The real ceiling is the WS2812 wire: 144 × 24 bits × 1.25 µs plus the latch ≈ 4.6 ms per frame, or about 215 fps. **Baud is ignored by USB CDC**, so any value configured in HyperHDR works.

**Realtime ownership** is stock WLED behaviour. Frames override the output while effects and segments keep their state underneath. When frames stop, `handleNotifications()` calls `exitRealtime()` after `realtimeTimeoutMs` (2500 ms default, inside the required 1–3 s window), which restores brightness and resumes the previous effect or preset. A truncated frame never calls `realtimeLock()`, so it cannot extend realtime mode. The parser re-syncs on the next `Ada` header.

### TXS0108E OE lifecycle

| Phase | What drives OE (GPIO7) |
|---|---|
| Power-up / reset / panic | GPIO is high-Z, so the **external 10 k pull-down** holds it LOW |
| C++ static init (before `setup()`, before WLED touches any pin) | usermod constructor: level 0 latched, then output enabled; internal pull-down on as backup |
| WLED reads config, `beginStrip()` (RMT bus created, black frame shown) | still LOW |
| `UsermodManager::setup()` | pin allocated in PinManager (`UM_Unspecified`); still LOW |
| First `loop()` where GPIO6 is owned by `PinOwner::BusDigital` **and** `!strip.isUpdating()` (no frame in flight) **and** optional `oeDelayMs` (default 0) has elapsed | **HIGH**, then `strip.trigger()` repaints, overwriting anything DIN latched while it floated |
| LED output removed or moved off GPIO6 at runtime | LOW again until it returns |
| OTA start (`onUpdateBegin(true)`) | LOW; re-enabled automatically if the update fails |
| `esp_restart()` (reboot, OTA finish) | LOW, via `esp_register_shutdown_handler` |

There is no fixed boot delay: the ESP's 3.3 V rail is stable before code runs, and the translator only needs an idle data line. A missing sensor or broker never holds OE low. If something else already owns GPIO7, OE **stays LOW** and Info reports a pin conflict.

### Pins and conflict checks

Every pin is set once, in the `[wled_assitant]` section of `platformio_override.ini`. From there they flow to WLED's defaults (`DATA_PINS`, `PIXEL_COUNTS`, `I2CSDAPIN`, `I2CSCLPIN`) and to the usermod (`WLEDA_*`). The usermod has `static_assert`s that reject strapping pins (0, 3, 45, 46), USB D−/D+ (19, 20), flash/PSRAM pins (22–32), UART0 (43, 44) and duplicate assignments. Verified: building with OE=20, OE=6, SDA=44 or DATA=0 each fails with a clear message. Plain `WLED_DEBUG` is also a build error, because it would log to the USB port HyperHDR uses. Use `WLED_DEBUG_HOST` (UDP) instead; it is commented out in the override. At runtime WLED's PinManager owns GPIO6 (LED bus), GPIO8/9 (`HW_I2C`) and GPIO7 (usermod).

The OE pin is **compile-time only**, because it must be driven before `cfg.json` is read. The LED pin, count, color order and I2C pins keep their normal WLED UI settings; the build only sets their defaults.

### HTU21D

The sensor is read with direct I2C transactions, so there is no library dependency. It uses no-hold-master mode with soft reset `0xFE`, then temperature `0xF3`, then humidity `0xF5`. Each step is one transaction of under 1 ms. The 50 ms and 16 ms conversion times are waited out across `loop()` calls, and a NACK means "still converting", so the sensor is polled again 10 ms later. Other details:

- **CRC-8** (polynomial 0x31) is checked on every reading. A host test reproduced the datasheet vectors: 0x683A → 0x7C and 0x4E85 → 0x6B.
- **Wire timeout** is set to 15 ms, so a stuck bus cannot stall the loop for long.
- **Polling** happens every 30 s by default (configurable 5–3600 s). The first reading comes 2 s after boot.
- **Error recovery:** after an error it retries in 5 s. After 3 consecutive failures, or a NACK at `0x40`, the sensor is marked missing and is re-probed every interval, so hot-plug recovery works.
- **Reported values:** readings are shown as stale after 3 missed intervals. Temperature and humidity offsets can be set in the usermod settings.

## Build

Requirements: `git`, Node.js ≥ 20, and PlatformIO (`uv tool install platformio`).

```sh
scripts/build.sh
# → wled/build_output/release/WLED_16.0.1_ESP32-S3_SuperMini_wled-assitant.bin
```

Manual equivalent: `scripts/bootstrap.sh && cd wled && npm run build && pio run -e wled_assitant_s3_supermini`.

The current build uses 76.5 % of the 1.5 MB OTA app slot and 14.1 % of static RAM. The partition table is upstream's `WLED_ESP32_4MB_1MB_FS.csv`: two 1.5 MB OTA slots plus a 960 KB filesystem.

## Flashing

**Stop HyperHDR's LED output first**, because only one program can hold the serial port. The port shows up as `/dev/cu.usbmodem*` on macOS, `/dev/ttyACM*` on Linux, or `COMx` on Windows.

First flash (erases old settings):

```sh
cd wled
pio run -e wled_assitant_s3_supermini -t erase  --upload-port /dev/cu.usbmodemXXXX
pio run -e wled_assitant_s3_supermini -t upload --upload-port /dev/cu.usbmodemXXXX
```

Equivalent esptool command (from `wled/`):

```sh
esptool.py --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 921600 write_flash \
  0x0     .pio/build/wled_assitant_s3_supermini/bootloader.bin \
  0x8000  .pio/build/wled_assitant_s3_supermini/partitions.bin \
  0xe000  ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 .pio/build/wled_assitant_s3_supermini/firmware.bin
```

For later updates, use the same `-t upload` command (settings are kept), or upload `build_output/release/*.bin` through WLED → Config → Security & Updates → Manual OTA.

**Recovery:** hold **BOOT**, tap **RST** (or plug in USB while holding BOOT), then flash. The ROM bootloader on the USB-Serial/JTAG port works regardless of what the firmware is doing.

## WLED first-boot configuration

1. Join the Wi-Fi network `WLED-AP` (password `wled1234`), open `http://4.3.2.1`, and enter your Wi-Fi. Alternatively, use Improv over the same USB port.
2. **Config → LED Preferences** (defaults from the build; check them):
   - Output: WS281x, GPIO **6**, length **144**, color order **GRB**. Change the order here if the colors are swapped; no rebuild is needed.
   - Current limiter: **enabled**, max **7000 mA**, **55 mA/LED**.
3. **Config → Usermods**:
   - Global I2C is SDA **9**, SCL **8**, set by default.
   - `wled_assitant` settings: interval 30 s, offsets 0, MQTT publish on, HA discovery on, discovery prefix `homeassistant`, oeDelayMs 0.
4. **Config → Sync Interfaces**:
   - **Uncheck "Receive UDP realtime"**, so USB is the only realtime source.
   - Realtime **Timeout 2500 ms** (default). Leave "Force max brightness" off.
   - MQTT: leave disabled for now.
5. **Config → User Interface**: "Device Name" defaults to "Desk Ambient Light", which is also the Home Assistant device name.
6. Optionally save a preset and set it as the boot preset under LED Preferences → "Apply preset at boot".

## HyperHDR configuration (Adalight over the same USB cable)

In **LED Hardware → LED Controller**:

| Setting | Value |
|---|---|
| Controller type | **adalight** |
| Output path | the ESP32's USB serial port (`/dev/cu.usbmodem…`, `/dev/ttyACM0`, `COMx`) |
| Protocol | **Ada** (not AWA; WLED only parses `Ada`) |
| Baudrate | any listed value, e.g. 2000000 (USB CDC ignores it) |
| ESP8266/ESP32 handshake | **off** (it toggles DTR/RTS, which resets the S3 through USB-Serial/JTAG) |
| White channel calibration | off (AWA only) |
| Delay after connect | 0 |
| Refresh time | **1000 ms** (must stay below WLED's 2500 ms timeout, or a static picture would hand control back to WLED) |
| RGB byte order | **RGB** (WLED converts to GRB itself via the LED Preferences color order) |
| Hardware LED count | **144** |

In **LED Hardware → LED Layout**, define the 144 LEDs to match how the strip is mounted. In **Image Processing → Smoothing**, set the update frequency to **60 Hz**; anything up to about 100 Hz has ample headroom.

When HyperHDR's LED device is switched off it sends a black frame and stops. WLED then returns to its previous effect after 2.5 s.

## Home Assistant

**Now (no broker):**
1. Settings → Devices & Services: the **WLED** integration auto-discovers the device over zeroconf; you can also add it by IP. This gives you the light, effects, presets and brightness.
2. Optional interim temperature and humidity entities, polled from WLED's JSON API (`configuration.yaml`):

   ```yaml
   rest:
     - resource: http://<wled-ip>/json/info
       scan_interval: 60
       sensor:
         - name: "Desk Ambient Light Temperature"
           unique_id: desk_ambient_light_temperature_rest
           value_template: "{{ value_json.wled_assitant.htu21d.temp_c | default(none) }}"
           unit_of_measurement: "°C"
           device_class: temperature
           state_class: measurement
         - name: "Desk Ambient Light Humidity"
           unique_id: desk_ambient_light_humidity_rest
           value_template: "{{ value_json.wled_assitant.htu21d.rh_pct | default(none) }}"
           unit_of_measurement: "%"
           device_class: humidity
           state_class: measurement
   ```

**Later (MQTT), with no firmware rebuild:**
1. Install a broker (e.g. the Mosquitto add-on) and the HA **MQTT** integration (discovery on, prefix `homeassistant`).
2. In WLED → Sync Interfaces → MQTT: enable it and set the broker host, port, user and password. Keep the default device topic `wled/<last-6-of-mac>`. Save.
3. On connect, the usermod publishes retained discovery configs to `homeassistant/sensor/wled_<mac>/{temperature,humidity}/config`, readings to `wled/<id>/htu21d/{temperature,humidity}` (°C and %, one decimal, retained, once per interval), and sensor availability to `wled/<id>/htu21d/status`. Entities use `availability_mode: all` together with WLED's own `wled/<id>/status` LWT. The discovery `device.connections` contains the MAC, so HA attaches the sensors to the **same device** as the WLED light. Remove the interim REST sensors at this point.
4. Turning off "HA discovery" in the usermod settings publishes empty retained configs, which removes the entities.

No voice-specific logic is in the firmware. Assist and voice pipelines use the WLED and sensor entities directly.

## First-power / test sequence

1. **Before connecting the strip:** with the board unpowered, measure OE to GND. It should read ≈0 V (pull-down present).
2. Flash over USB with the LED PSU off. Open `http://<ip>/json/info`. Check that `wled_assitant.oe.enabled` is true and `enabled_at_ms` is a boot-time value (well under 1–2 s after boot). Check that `u["USB Adalight"]` reads `idle`, not "disabled".
3. **Scope or logic analyser** on OE (and optionally on 3V3 and GPIO6), triggered on RST:
   - OE stays LOW through reset and boot.
   - OE goes HIGH only after the boot frame, with no GPIO6 activity at the moment it rises.
   - Pressing RST or Config → Reboot pulls OE LOW immediately.
4. Power the LED PSU and confirm all 144 LEDs work with WLED effects, presets and segments. Set a full-white effect at full brightness and confirm Info shows the current estimate capped near 7 A, with brightness reduced.
5. **USB Adalight without HyperHDR** (from the project root):

   ```sh
   uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodemXXXX --probe        # prints "WLED 16.0.1 ..."
   uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodemXXXX --fps 60 --seconds 30
   uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodemXXXX --malformed --seconds 15
   ```

   - **During the stream:** you see a moving rainbow, and Info shows "USB Adalight: streaming".
   - **After it stops:** the previous effect returns about 2.5 s later.
   - **`--malformed`:** at most an occasional glitched frame and no lock-up. WLED still recovers 2.5 s after the run ends.
   - **Uptime:** WLED's uptime must not reset when the port opens. If it does, see the DTR/RTS note under Known limitations.
6. **Under load:** start HyperHDR with the settings above and play full-screen motion for 10 minutes or more. Hammer the WLED web UI at the same time and look for stutter.
7. Unplug and replug USB while HyperHDR runs. HyperHDR reconnects, and WLED needs no reboot because the port is hardware-backed.
8. **HTU21D:** `wled_assitant.htu21d.status` should be `ok` with plausible values, and `age_s` should cycle up to 30. Unplug the sensor: the status becomes `HTU21D not found at 0x40` within about 40 s while LEDs and USB keep working. Replug it and readings return within one interval.
9. **Failure modes:**
   - Wrong Wi-Fi password: the LEDs, USB stream and sensor still work, and WLED opens its AP.
   - MQTT enabled with an unreachable broker: no effect on USB streaming.
   - None of these cause a reboot.

## Known limitations / open points

- **Not yet hardware-tested.** Everything above is from source analysis plus compile checks.
- **No USB flow control on HWCDC.** Bytes beyond the 2 KB queue are dropped, which needs the WLED loop to stall for more than about 75 ms at 60 fps; saving config or presets to flash can do this. The stock parser then shows one bad frame and re-syncs on the next `Ada` header. Raise `WLEDA_USB_RX_BUFFER` if this is ever visible.
- **Stock parser quirks.** In its idle state, WLED's serial parser also treats bytes such as `{`, `v`, `I`, `0xB0–0xB7` as commands (JSON API, version, Improv, baud). After a desync, frame data can momentarily be read as one of these; the worst case is a parse of up to 100 ms, then recovery. Changing this would need a core patch, which this project rules out.
- **DTR/RTS reset.** On the USB-Serial/JTAG port, the host can reset the chip by asserting RTS while DTR is low. Normal port opens (HyperHDR, pyserial defaults, `cat`) do not do this. Tools that toggle the lines (HyperHDR's ESP handshake, some terminal programs) will reboot WLED.
- **DIN floats while OE is LOW.** The TXS0108E B-side is high-Z then. If random pixels light on power-up before the first repaint, add a 10–100 k pull-down from DIN to GND on the strip side of the 220 Ω resistor.
- **TXS0108E drive strength.** It is an auto-direction translator with weak drive and is marginal for WS2812 over long leads. If the first pixel flickers, try a short data lead or a 74AHCT125/SN74HCT245 buffer.
- **SuperMini variants.** Clones differ. `esptool.py flash_id` should report 4 MB flash, and the boot log or Info should show 2 MB PSRAM. If PSRAM is absent, WLED still runs, falling back to internal RAM. `LOLIN_WIFI_FIX` starts Wi-Fi TX power at 8.5 dBm for the poor on-board antennas; raise it in Config → WiFi if the link is weak. Some boards have an on-board RGB LED on GPIO48, which is unused here.
- **Sensor accuracy.** The HTU21D reads the air around it; mount it away from the ESP32, the PSU and the LEDs, or use `tempOffset`.
