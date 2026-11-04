<div id="top">
    <div align="center">
        <h1 align='center'>wled-assitant</h1>
        <strong>Desk ambient light firmware: unmodified upstream WLED on an ESP32-S3 SuperMini, fed by HyperHDR over USB Adalight.</strong>
    </div>
</div>

______________________________________________________________________

<div align='center'>
    <a href='https://github.com/DivitMittal/wled-assitant'>
        <img src='https://img.shields.io/github/repo-size/DivitMittal/wled-assitant?&style=for-the-badge&logo=github' alt="size"/>
    </a>
    <a href='https://github.com/wled/WLED/releases/tag/v16.0.1'>
        <img src='https://img.shields.io/static/v1.svg?style=for-the-badge&label=WLED&message=v16.0.1&logo=espressif' alt="wled"/>
    </a>
    <img src='https://img.shields.io/static/v1.svg?style=for-the-badge&label=Board&message=ESP32--S3%20SuperMini&logo=espressif' alt="board"/>
</div>

______________________________________________________________________

<div align='center'>
    <a href="https://github.com/DivitMittal/wled-assitant/actions/workflows/flake-check.yml">
        <img src="https://github.com/DivitMittal/wled-assitant/actions/workflows/flake-check.yml/badge.svg" alt="nix-flake-check"/>
    </a>
    <a href="https://github.com/DivitMittal/wled-assitant/actions/workflows/flake-lock-update.yml">
        <img src="https://github.com/DivitMittal/wled-assitant/actions/workflows/flake-lock-update.yml/badge.svg" alt="flake-lock-update"/>
    </a>
</div>

______________________________________________________________________

## Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Build and Flash](#build-and-flash)
- [Setup](#setup)
  - [WLED](#wled)
  - [HyperHDR](#hyperhdr)
  - [Home Assistant](#home-assistant)
- [First-Power Test](#first-power-test)
- [Known Limitations](#known-limitations)
- [Related Repositories](#related-repositories)
- [For AI Agents](#for-ai-agents)

______________________________________________________________________

## Overview

An ESP32-S3 SuperMini (ESP32-S3FH4R2, 4 MB flash / 2 MB PSRAM) running **unmodified upstream [WLED](https://github.com/wled/WLED) v16.0.1** plus one usermod:

- Drives **144 WS2812B LEDs** directly from a 3.3V GPIO through a 220Ω series resistor. There is no level shifter; the LED PSU is trimmed to 4.75V so the strip reliably reads 3.3V logic.
- Takes [HyperHDR](https://github.com/awawa-dev/HyperHDR) frames as **Adalight over the board's own USB-C port**.
- Reads an **HTU21D** temperature/humidity sensor.
- Keeps Wi-Fi on for the Web UI, Home Assistant, OTA and optional MQTT.

> **Status:** LED output confirmed working on hardware with the direct 3.3V-GPIO-drive wiring (2026-10-08). Other subsystems (HTU21D, MQTT/HA, full HyperHDR streaming) are still untested on hardware — follow the [first-power test](#first-power-test) before relying on them.

## Project Structure

```
./
├── platformio_override.ini     # build env; the only place pins, LED count and current limit are set
├── usermods/wled_assitant/     # the usermod (HTU21D, MQTT/HA discovery, diagnostics)
├── scripts/
│   ├── bootstrap.sh            # clones WLED v16.0.1 into wled/ and refuses a modified core
│   └── build.sh                # bootstrap + web UI + firmware build
├── tools/adalight_test.py      # sends Adalight frames over USB without HyperHDR
├── flake/                      # flake-parts modules: devshells, formatters, checks, CI actions
├── .apm/instructions/          # design notes and agent rules (compiled by APM)
├── apm.yml                     # APM manifest (agent rules, MCP servers)
├── wled/                       # upstream checkout (gitignored, never edited)
└── flake.nix                   # entry point — devshell with PlatformIO, esptool, Node.js, …
```

______________________________________________________________________

## How It Works

- **USB Adalight:** with `ARDUINO_USB_MODE=1` and `ARDUINO_USB_CDC_ON_BOOT=1`, WLED's `Serial` is the S3's hardware USB-Serial/JTAG port, so WLED's stock Adalight parser reads HyperHDR directly. The usermod enlarges the port's receive buffer from 256 B to 2 KB, because the driver silently drops bytes when it fills and one frame is 438 B. 60 fps needs 26 kB/s, a small fraction of USB full speed. When frames stop, WLED returns to the previous effect after its 2.5 s realtime timeout.
- **Direct LED drive:** DIN is fed straight from GPIO5 (3.3V logic) through a 220Ω series resistor — no level shifter. The LED PSU is trimmed to 4.75V (instead of 5V) so the strip's data-high threshold sits reliably below 3.3V; this is a hardware/voltage choice, nothing for the firmware to control.
- **HTU21D (SDA 13 / SCL 12):** read with direct I2C commands, no library, without blocking the loop. CRC-checked, every 30 s, recovers after unplugging.
- **MQTT:** optional. Once WLED's MQTT is configured, readings and Home Assistant discovery are published and attached to the same device as the WLED light.
- **Safety nets:** reserved or duplicate pins fail the build (`static_assert`); plain `WLED_DEBUG` is a build error because it would log onto the HyperHDR port (use `WLED_DEBUG_HOST`).

## Build and Flash

`nix develop` (auto-activated via direnv) provides PlatformIO, esptool, Node.js, tio, uv and clangd, plus these commands (`menu` lists them):

| Command | Description |
|---|---|
| `fw-build` | bootstrap pinned WLED and build |
| `fw-flash <port>` | build and flash over USB |
| `fw-erase <port>` | erase all flash, including WLED settings (do this before the first flash) |
| `fw-info <port>` | chip, flash size and PSRAM check |
| `fw-monitor <port>` | serial console that doesn't toggle DTR/RTS |
| `fw-ports` | list ESP32-S3 USB ports |
| `wt-add <name>` / `wt-list` / `wt-rm <name>` | local worktrees (jj workspaces) in `.worktrees/` |

Without nix: install Node.js ≥ 20 and PlatformIO, then run `scripts/build.sh` (add `upload --upload-port <port>` to flash). The image lands in `wled/build_output/release/` and uses about 77 % of the 1.5 MB OTA slot. Later updates also work through WLED's Manual OTA page.

> ⚠️ **Stop HyperHDR's LED output before flashing**, since only one program can hold the port. **Recovery:** hold BOOT, tap RST, then flash.

______________________________________________________________________

## Setup

### WLED

Join `WLED-AP` / `wled1234`, open `http://4.3.2.1`, and enter your Wi-Fi.

- **LED Preferences:** GPIO 5, 144 LEDs, GRB, current limiter 7000 mA. These are build defaults; fix the color order here if colors look swapped.
- **Sync Interfaces:** uncheck **Receive UDP realtime**, so USB is the only realtime source. Keep the timeout at 2500 ms.

### HyperHDR

Under **LED Hardware**:

| Setting | Value |
|---|---|
| Controller / protocol | **adalight**, **Ada** (not AWA) |
| Output path | the ESP32's port (`/dev/cu.usbmodem…`, `/dev/ttyACM0`, `COMx`) |
| Baud | any (ignored by USB) |
| ESP8266/ESP32 handshake | **off** (it toggles DTR/RTS, which resets the S3) |
| Refresh time | **1000 ms** (must stay below WLED's 2.5 s timeout) |
| RGB order / LED count | **RGB** / **144** |

Set the smoothing update frequency to 60 Hz.

### Home Assistant

- The **WLED** integration auto-discovers the light.
- Until you have an MQTT broker, temperature and humidity can come from a REST sensor on `http://<wled-ip>/json/info` (`value_json.wled_assitant.htu21d.temp_c` and `.rh_pct`).
- Later: add a broker and HA's MQTT integration, then enable MQTT in WLED → Sync Interfaces. The sensors then appear on the same device via discovery, with no rebuild needed.

## First-Power Test

1. Flash with the LED PSU off. In `/json/info`, "USB Adalight" should read `idle`.
1. Power the strip: check effects, presets, and that full white is capped near 7 A.
1. Test USB without HyperHDR:
   ```sh
   uv run --with pyserial tools/adalight_test.py <port> --probe          # prints "WLED 16.0.1 ..."
   uv run --with pyserial tools/adalight_test.py <port> --fps 60 --seconds 30
   uv run --with pyserial tools/adalight_test.py <port> --malformed --seconds 15
   ```
   You should see a rainbow; WLED resumes its effect about 2.5 s after the stream stops, and its uptime must not reset when the port opens.
1. Run HyperHDR for 10+ minutes while using the web UI; unplug and replug USB.
1. Unplug the HTU21D: within about 40 s Info reports it missing while LEDs keep working; readings return after replugging.

## Known Limitations

- **Partly hardware-tested:** LED output is confirmed; HTU21D, MQTT/HA and sustained HyperHDR streaming are not yet.
- **No USB flow control:** a WLED stall over about 75 ms (e.g. saving settings) can cause one bad frame before the parser resyncs.
- **DTR/RTS:** tools that drop DTR while asserting RTS reset the chip. Normal port opens don't.
- **DIN floats at power-on** (GPIO5 isn't driven until WLED's strip init runs, ~1 s after power-on): if stray pixels light briefly, add a 10–100 kΩ pull-down on DIN.
- **Out-of-spec logic level:** driving WS2812 DIN from 3.3V (even with the PSU trimmed to 4.75V) is below the WS2812 datasheet's nominal 0.7×VDD threshold. It works on this strip/PSU combination as tested, but isn't guaranteed across all strip batches, cable lengths, or temperatures — if it becomes unreliable, the fallback is a level shifter (e.g. 74AHCT125) between GPIO5 and DIN.
- **SuperMini clones vary:** check `fw-info`. Wi-Fi TX power starts at 8.5 dBm (`LOLIN_WIFI_FIX`); raise it in Config → WiFi if the link is weak.

______________________________________________________________________

## Related Repositories

- [wled/WLED](https://github.com/wled/WLED): Upstream firmware, pinned at v16.0.1 and never patched
- [awawa-dev/HyperHDR](https://github.com/awawa-dev/HyperHDR): Screen-capture ambient lighting host that streams the Adalight frames
- [DivitMittal/infra-nixCfg](https://github.com/DivitMittal/infra-nixCfg): Main Nix configurations repository

## For AI Agents

Context files (`AGENTS.md`, `CLAUDE.md`) are generated — not committed. Run `apm compile` before exploring the repo to get directory-level guidance.

<div align="right">

[![][back-to-top]](#top)

</div>

[back-to-top]: https://img.shields.io/badge/-BACK_TO_TOP-151515?style=flat-square
