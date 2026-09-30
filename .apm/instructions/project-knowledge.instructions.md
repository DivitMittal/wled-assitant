---
description: Project-wide knowledge base for wled-assitant (ESP32-S3 WLED desk ambient light)
applyTo: "**"
---

# PROJECT KNOWLEDGE BASE

## OVERVIEW

Custom firmware = **unmodified upstream WLED v16.0.1** + one usermod. ESP32-S3 SuperMini (ESP32-S3FH4R2: 4 MB flash, 2 MB quad PSRAM) drives 144 WS2812B LEDs through a TXS0108E level shifter, receives HyperHDR frames as **Adalight over the board's native USB-C port**, reads an HTU21D, and keeps Wi-Fi for Web UI / JSON API / HA WLED integration / OTA / optional MQTT.

## STRUCTURE

```
./
├── platformio_override.ini     # build env + single source of truth for pins/LED count/ABL (symlinked into wled/)
├── usermods/wled_assitant/     # the only custom firmware code (library.json + wled_assitant.cpp)
├── scripts/bootstrap.sh        # clone WLED v16.0.1 → wled/, fail on core modifications, link override, npm ci
├── scripts/build.sh            # bootstrap + `npm run build` + `pio run` (`build.sh upload` flashes)
├── tools/adalight_test.py      # send Adalight frames over USB without HyperHDR
├── wled/                       # upstream checkout — gitignored, NEVER edit
├── .apm/instructions/          # agent memory sources (compiled by APM)
└── flake.nix, flake/           # devshell (provides `apm`), treefmt, git-hooks
```

## HARD CONSTRAINTS

- **Zero upstream WLED core patches.** Everything goes through `custom_usermods`, usermod hooks, build flags, or public framework APIs. If something truly needs a core patch, stop and report the blocker.
- HyperHDR transport is **Adalight over USB CDC only** — never DDP/E1.31/Art-Net/UDP/Wi-Fi, never AWA/TPM2/HyperSerialWLED.
- No second LED renderer, no FastLED/NeoPixel code, no task writing LEDs concurrently with WLED.
- Never log to `Serial` (it is the HyperHDR port). Plain `WLED_DEBUG` is a deliberate `#error`; use `WLED_DEBUG_HOST` (UDP, `nc -kul 7868`).
- MQTT, Wi-Fi, HTU21D, HyperHDR are all optional at runtime; none may block boot, OE enable, or USB streaming.

## FIXED HARDWARE

| Signal | GPIO | Notes |
| --- | --- | --- |
| LED data | 6 | → TXS0108E A7 → B7 → 220 Ω → DIN |
| TXS0108E OE | 7 | external 10 k pull-down; compile-time only |
| HTU21D SCL / SDA | 8 / 9 | WLED global I2C; addr 0x40; breakout may have pull-ups |
| USB D−/D+ | 19 / 20 | USB-Serial/JTAG — reserved |
| UART0 TX/RX | 43 / 44 | WLED disables serial RX (Adalight) if 44 is allocated |

Defaults: 144 LEDs, GRB, ABL 7000 mA @ 55 mA/LED, realtime timeout 2500 ms, HTU21D 30 s, device name "Desk Ambient Light".

## COMMANDS

```bash
nix develop                                    # devshell (apm, nix tooling)
scripts/build.sh                               # full build → wled/build_output/release/*.bin
scripts/build.sh upload --upload-port /dev/cu.usbmodemXXXX
cd wled && pio run -e wled_assitant_s3_supermini -t compiledb   # compile_commands.json for clangd/serena
uv run --with pyserial tools/adalight_test.py <port> --probe     # "WLED 16.0.1 ..." over USB
apm install && apm compile                     # regenerate agent rules / MCP config from .apm + apm.yml
```

Build env name: `wled_assitant_s3_supermini` (extends upstream `esp32s3_4M_qspi`, board `lolin_s3_mini`). Validation = successful compile; there are no C++ unit tests. Flash usage is ~76.5 % of the 1.5 MB OTA slot — watch it.

## VCS

Colocated **jj** on git (`jj st`, `jj describe`, `jj new`, `jj git push`). `wled/` is gitignored; pin upgrades go through `WLED_TAG` in `scripts/bootstrap.sh`.

## NOTES

- Spelling is intentional: project/usermod name is **`wled-assitant`** / `wled_assitant` (the directory is `wled-assistant`).
- `README.md` holds user-facing docs (flashing, HyperHDR, Home Assistant, test checklist, limitations). Keep it in sync with behaviour changes.
- Nothing has been hardware-tested yet; claims about runtime behaviour come from source analysis.
