# wled-assitant

Desk ambient light firmware: an ESP32-S3 SuperMini (ESP32-S3FH4R2, 4 MB flash / 2 MB PSRAM) running **unmodified upstream WLED v16.0.1** plus one usermod. It drives 144 WS2812B LEDs through a TXS0108E level shifter, takes HyperHDR frames as **Adalight over the board's own USB-C port**, and reads an HTU21D sensor. Wi-Fi stays on for the Web UI, Home Assistant, OTA and optional MQTT.

> **Status:** compiles and links; logic was checked against the WLED and arduino-esp32 source. **Not yet run on hardware.** Follow the [test sequence](#first-power-test) before relying on it.

## Layout

```
platformio_override.ini     build env; the only place pins, LED count and current limit are set
usermods/wled_assitant/     the usermod (OE control, HTU21D, MQTT/HA discovery, diagnostics)
scripts/bootstrap.sh        clones WLED v16.0.1 into wled/ and refuses a modified core
scripts/build.sh            bootstrap + web UI + firmware build
tools/adalight_test.py      sends Adalight frames over USB without HyperHDR
.apm/instructions/          design notes and agent rules (compiled by APM)
wled/                       upstream checkout (gitignored, never edited)
```

## How it works

- **USB Adalight:** with `ARDUINO_USB_MODE=1` and `ARDUINO_USB_CDC_ON_BOOT=1`, WLED's `Serial` is the S3's hardware USB-Serial/JTAG port, so WLED's stock Adalight parser reads HyperHDR directly. The usermod enlarges the port's receive buffer from 256 B to 2 KB, because the driver silently drops bytes when it fills and one frame is 438 B. 60 fps needs 26 kB/s, a small fraction of USB full speed. When frames stop, WLED returns to the previous effect after its 2.5 s realtime timeout.
- **Level shifter OE (GPIO7):** held LOW by the external pull-down until the usermod drives it LOW during static init, before WLED touches any pin. It goes HIGH once WLED's LED output owns GPIO6 and no frame is in flight. It drops LOW again during OTA, on reboot, or if the LED output moves.
- **HTU21D (SDA 9 / SCL 8):** read with direct I2C commands, no library, without blocking the loop. CRC-checked, every 30 s, recovers after unplugging.
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

Without nix: install Node.js ≥ 20 and PlatformIO, then run `scripts/build.sh` (add `upload --upload-port <port>` to flash). The image lands in `wled/build_output/release/` and uses about 77 % of the 1.5 MB OTA slot. Later updates also work through WLED's Manual OTA page.

**Stop HyperHDR's LED output before flashing**, since only one program can hold the port. **Recovery:** hold BOOT, tap RST, then flash.

## Setup

**WLED** (join `WLED-AP` / `wled1234`, open `http://4.3.2.1`, enter your Wi-Fi):
- LED Preferences: GPIO 6, 144 LEDs, GRB, current limiter 7000 mA. These are build defaults; fix the color order here if colors look swapped.
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
- Until you have an MQTT broker, temperature and humidity can come from a REST sensor on `http://<wled-ip>/json/info` (`value_json.wled_assitant.htu21d.temp_c` and `.rh_pct`).
- Later: add a broker and HA's MQTT integration, then enable MQTT in WLED → Sync Interfaces. The sensors then appear on the same device via discovery, with no rebuild needed.

## First-power test

1. With the board unpowered, OE to GND should read ≈0 V.
2. Flash with the LED PSU off. In `/json/info`, `wled_assitant.oe.enabled` should be true and "USB Adalight" should read `idle`.
3. With a scope on OE: it stays LOW through reset and boot, rises only after the boot frame, and drops on RST.
4. Power the strip: check effects, presets, and that full white is capped near 7 A.
5. Test USB without HyperHDR:
   ```sh
   uv run --with pyserial tools/adalight_test.py <port> --probe          # prints "WLED 16.0.1 ..."
   uv run --with pyserial tools/adalight_test.py <port> --fps 60 --seconds 30
   uv run --with pyserial tools/adalight_test.py <port> --malformed --seconds 15
   ```
   You should see a rainbow; WLED resumes its effect about 2.5 s after the stream stops, and its uptime must not reset when the port opens.
6. Run HyperHDR for 10+ minutes while using the web UI; unplug and replug USB.
7. Unplug the HTU21D: within about 40 s Info reports it missing while LEDs keep working; readings return after replugging.

## Known limitations

- **Not hardware-tested yet.**
- **No USB flow control:** a WLED stall over about 75 ms (e.g. saving settings) can cause one bad frame before the parser resyncs.
- **DTR/RTS:** tools that drop DTR while asserting RTS reset the chip. Normal port opens don't.
- **DIN floats while OE is LOW:** if stray pixels light at power-on, add a 10–100 kΩ pull-down on DIN.
- **TXS0108E** has weak drive for WS2812; if the first pixel flickers, use a short lead or a 74AHCT125.
- **SuperMini clones vary:** check `fw-info`. Wi-Fi TX power starts at 8.5 dBm (`LOLIN_WIFI_FIX`); raise it in Config → WiFi if the link is weak.
