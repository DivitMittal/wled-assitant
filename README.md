# wled-assitant

Desk ambient light firmware: an ESP32-S3 SuperMini (ESP32-S3FH4R2, 4 MB flash / 2 MB PSRAM) running **unmodified upstream WLED v16.0.1** plus one usermod. It drives 144 WS2812B LEDs directly from a 3.3V GPIO through a 220Ω series resistor (no level shifter; the LED PSU is trimmed to 4.75V so the strip reliably reads 3.3V logic), takes HyperHDR frames as **Adalight over the board's own USB-C port**, and reads a BMP280 (temperature / pressure) and a DHT11 (humidity). Wi-Fi stays on for the Web UI, Home Assistant, OTA and optional MQTT.

> **Status:** LED output (2026-10-08) and BMP280 readings (2026-10-30) confirmed working on hardware. DHT11 humidity and dew point confirmed on hardware 2026-11-09. MQTT/HA and full HyperHDR streaming are still untested on hardware — follow the [test sequence](#first-power-test) before relying on them.

## Layout

```
platformio_override.ini     build env; the only place pins, LED count and current limit are set
usermods/wled_assitant/     the usermod (BMP280, DHT11, MQTT/HA discovery, diagnostics)
scripts/bootstrap.sh        clones WLED v16.0.1 into wled/ and refuses a modified core
scripts/build.sh            bootstrap + web UI + firmware build
tools/adalight_test.py      sends Adalight frames over USB without HyperHDR
.apm/instructions/          design notes and agent rules (compiled by APM)
wled/                       upstream checkout (gitignored, never edited)
```

## How it works

- **USB Adalight:** with `ARDUINO_USB_MODE=1` and `ARDUINO_USB_CDC_ON_BOOT=1`, WLED's `Serial` is the S3's hardware USB-Serial/JTAG port, so WLED's stock Adalight parser reads HyperHDR directly. The usermod enlarges the port's receive buffer from 256 B to 2 KB, because the driver silently drops bytes when it fills and one frame is 438 B. 60 fps needs 26 kB/s, a small fraction of USB full speed. When frames stop, WLED returns to the previous effect after its 2.5 s realtime timeout.
- **Direct LED drive:** DIN is fed straight from GPIO5 (3.3V logic) through a 220Ω series resistor — no level shifter. The LED PSU is trimmed to 4.75V (instead of 5V) so the strip's data-high threshold sits reliably below 3.3V; this is a hardware/voltage choice, nothing for the firmware to control.
- **BMP280 (SDA 12 / SCL 3, address 0x76 or 0x77):** temperature and pressure (no humidity), read with direct I2C commands, no library, without blocking the loop, every 30 s; recovers after unplugging. Any other chip is rejected, and `last_fault` names its ID (`id 60 @76, not BMP280`). `/json/info` → `wled_assitant.bmp280` has `temp_c`, `hpa` (station pressure, not sea-level), the raw ADC values and, after a failed read, `last_fault`.
- **BMP280 wiring:** on the common "5V" breakouts (on-board regulator + I2C level-shifting MOSFETs), power **VIN from 3V3, not 5V** — the MOSFETs pull SDA/SCL up to VIN, and the S3 isn't 5V tolerant. Keep the sensor away from the ESP32, PSU and strip, and don't clean it with solvents (that killed the humidity channel of the HTU21D it replaced).
- **DHT11 (DATA on GPIO1):** humidity. The firmware holds the line low for 20 ms between loop passes, then an RMT receive channel captures the reply in hardware, so nothing blocks and no interrupts are disabled (a bit-banged read would stall the LED driver). Reads every 30 s with the BMP280; three failed reads mark it missing, and it keeps retrying, so it can be connected later without reflashing. `/json/info` → `wled_assitant.dht11` has `rh_pct`, the DHT11's own `temp_c`, the `raw` frame and, after a failed read, `last_fault` (`no response`, `checksum`, `not a DHT11` for a DHT22, …).
- **DHT11 wiring:** VCC to **3V3, not 5V** (the module's pull-up would pull GPIO1 to 5V), GND to GND, DATA to GPIO1. 3-pin modules already have the pull-up; a bare 4-pin sensor needs 10 kΩ from DATA to 3V3. Keep the lead under about 1 m at 3.3V.
- **How the two work together:** temperature comes from the BMP280 (±0.5 °C, against the DHT11's ±2 °C) and humidity from the DHT11 (±5 %RH). Dew point is computed from both and only shown when both are fresh. Relative humidity is measured at the DHT11's own temperature, so mount the two sensors side by side, away from heat sources; if the DHT11's `temp_c` keeps reading several degrees off the BMP280, one of them is being heated. `humOffset` (Usermods settings) corrects a constant humidity bias.
- **MQTT:** optional. Once WLED's MQTT is configured, readings and Home Assistant discovery are published and attached to the same device as the WLED light.
- **Safety nets:** reserved or duplicate pins fail the build (`static_assert`); plain `WLED_DEBUG` is a build error because it would log onto the HyperHDR port (use `WLED_DEBUG_HOST`).

## Build and flash

`nix develop` provides PlatformIO, esptool, Node.js, tio, uv and clangd, plus these commands (`menu` lists them):

| Command | Does |
|---|---|
| `fw-build` | bootstrap pinned WLED and build |
| `fw-flash <port>` | build and flash over USB |
| `fw-erase <port>` | erase all flash, including WLED settings (do this before the first flash) |
| `fw-info <port>` | chip, flash size and PSRAM check |
| `fw-monitor <port>` | serial console that doesn't toggle DTR/RTS |
| `fw-ports` | list ESP32-S3 USB ports |
| `wt-add <name>` / `wt-list` / `wt-rm <name>` | local worktrees (jj workspaces) in `.worktrees/` |

Without nix: install Node.js ≥ 20 and PlatformIO, then run `scripts/build.sh` (add `upload --upload-port <port>` to flash). The image lands in `wled/build_output/release/` and uses about 77 % of the 1.5 MB OTA slot. Later updates also work through WLED's Manual OTA page.

**Stop HyperHDR's LED output before flashing**, since only one program can hold the port. **Recovery:** hold BOOT, tap RST, then flash.

## Setup

**WLED** (join `WLED-AP` / `wled1234`, open `http://4.3.2.1`, enter your Wi-Fi):
- LED Preferences: GPIO 5, 144 LEDs, GRB, current limiter 7000 mA. These are build defaults; fix the color order here if colors look swapped.
- Sync Interfaces: uncheck **Receive UDP realtime**, so USB is the only realtime source. Keep the timeout at 2500 ms.

**HyperHDR**, under LED Hardware:

| Setting | Value |
|---|---|
| Controller / protocol | **adalight**, **Ada** (not AWA) |
| Output path | the ESP32's port (`/dev/cu.usbmodem…`, `/dev/ttyACM0`, `COMx`) |
| Baud | any (ignored by USB) |
| ESP8266/ESP32 handshake | **off** (it toggles DTR/RTS, which resets the S3) |
| Refresh time | **1000 ms** (must stay below WLED's 2.5 s timeout) |
| RGB order / LED count | **RGB** / **144** |

Set the smoothing update frequency to 60 Hz.

**Home Assistant:**
- The **WLED** integration auto-discovers the light.
- Until you have an MQTT broker, readings can come from a REST sensor on `http://<wled-ip>/json/info` (`value_json.wled_assitant.bmp280.temp_c` and `.hpa`, `value_json.wled_assitant.dht11.rh_pct`, `value_json.wled_assitant.dew_point_c`).
- Later: add a broker and HA's MQTT integration, then enable MQTT in WLED → Sync Interfaces. Temperature, pressure, humidity and dew point then appear on the same device via discovery, with no rebuild needed. Each one goes unavailable when its sensor is missing; dew point needs both.

## First-power test

1. Flash with the LED PSU off. In `/json/info`, "USB Adalight" should read `idle`.
2. Power the strip: check effects, presets, and that full white is capped near 7 A.
3. Test USB without HyperHDR:
   ```sh
   uv run --with pyserial tools/adalight_test.py <port> --probe          # prints "WLED 16.0.1 ..."
   uv run --with pyserial tools/adalight_test.py <port> --fps 60 --seconds 30
   uv run --with pyserial tools/adalight_test.py <port> --malformed --seconds 15
   ```
   You should see a rainbow; WLED resumes its effect about 2.5 s after the stream stops, and its uptime must not reset when the port opens.
4. Run HyperHDR for 10+ minutes while using the web UI; unplug and replug USB.
5. Unplug the BMP280: within about 40 s Info reports it missing while LEDs keep working; readings return after replugging.
6. With the DHT11 connected, Info shows Humidity and Dew point within 30 s, and its `temp_c` is within a few degrees of the BMP280. Unplug it: Humidity reads "DHT11 not responding" within about 40 s, then recovers on replug. Repeat step 3 while it reads, to confirm the stream is unaffected.

## Known limitations

- **Not hardware-tested yet:** MQTT/HA and long HyperHDR sessions.
- **No USB flow control:** a WLED stall over about 75 ms (e.g. saving settings) can cause one bad frame before the parser resyncs.
- **DTR/RTS:** tools that drop DTR while asserting RTS reset the chip. Normal port opens don't.
- **DIN floats at power-on** (GPIO5 isn't driven until WLED's strip init runs, ~1 s after power-on): if stray pixels light briefly, add a 10–100 kΩ pull-down on DIN.
- **Out-of-spec logic level:** driving WS2812 DIN from 3.3V (even with the PSU trimmed to 4.75V) is below the WS2812 datasheet's nominal 0.7×VDD threshold. It works on this strip/PSU combination as tested, but isn't guaranteed across all strip batches, cable lengths, or temperatures — if it becomes unreliable, the fallback is a level shifter (e.g. 74AHCT125) between GPIO5 and DIN.
- **SuperMini clones vary:** check `fw-info`. Wi-Fi TX power starts at 8.5 dBm (`LOLIN_WIFI_FIX`); raise it in Config → WiFi if the link is weak.
