---
description: Project-wide knowledge base for wled-assitant (ESP32-S3 WLED desk ambient light)
applyTo: "**"
---

# PROJECT KNOWLEDGE BASE

## OVERVIEW

Custom firmware = **unmodified upstream WLED v16.0.1** + one usermod. ESP32-S3 SuperMini (ESP32-S3FH4R2: 4 MB flash, 2 MB quad PSRAM) drives 144 WS2812B LEDs directly from a 3.3V GPIO through a 220Ω series resistor (no level shifter; LED PSU trimmed to 4.75V so the strip reliably reads 3.3V logic — confirmed working on hardware 2026-10-08), receives HyperHDR frames as **Adalight over the board's native USB-C port**, reads a BME280 (temperature / humidity / pressure), and keeps Wi-Fi for Web UI / JSON API / HA WLED integration / OTA / optional MQTT.

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
- MQTT, Wi-Fi, the sensor, HyperHDR are all optional at runtime; none may block boot or USB streaming.

## FIXED HARDWARE

| Signal | GPIO | Notes |
| --- | --- | --- |
| LED data | 5 | → 220 Ω → DIN, direct 3.3V logic; no level shifter (TXS0108E removed) |
| BME280 SCL / SDA | 12 / 13 | WLED global I2C; addr 0x76 or 0x77; breakout may have pull-ups. 5V boards: VIN on 3V3 (their MOSFET shifters pull SDA/SCL to VIN) |
| USB D−/D+ | 19 / 20 | USB-Serial/JTAG — reserved |
| UART0 TX/RX | 43 / 44 | WLED disables serial RX (Adalight) if 44 is allocated |

LED PSU is trimmed to **4.75V** (not 5V) so the strip's data-high threshold reliably reads a direct 3.3V GPIO signal — measured/confirmed working 2026-10-08. Out of spec per the WS2812 datasheet (nominal threshold 0.7×VDD); works on the tested strip/PSU, not guaranteed for other batches — fallback is a level shifter (e.g. 74AHCT125) between GPIO5 and DIN if it becomes unreliable.

Defaults: 144 LEDs, GRB, ABL 7000 mA @ 55 mA/LED, realtime timeout 2500 ms, sensor 30 s, device name "Desk Ambient Light".

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

### Local worktrees (`.worktrees/`, gitignored)

- Worktrees are **jj workspaces**, not git worktrees: `wt-add <name> [rev]`, `wt-list`, `wt-rm <name>` (devshell). Parallel agents/tasks go in `.worktrees/<name>`.
- Why not `git worktree`: jj run inside a git worktree resolves to the *main* workspace and would commit its working copy; jj also refuses to colocate inside a git worktree.
- A workspace has no `.git`, so Nix rejects its own flake ("not tracked"). `.envrc` therefore loads the **main checkout's** flake there; test flake changes from the main checkout.
- Each workspace has its own `wled/` (and `.pio`), cloned locally from the main checkout's `wled/` by `bootstrap.sh` (a shallow repo can't be a `--reference`). The first build re-downloads PlatformIO libraries.
- `fw-*` commands act on the current workspace (`jj workspace root`), `wt-*` on the main checkout. `wt-rm` snapshots the workspace before forgetting it, so its changes stay in `jj log`.

## NOTES

- Spelling is intentional: project/usermod name is **`wled-assitant`** / `wled_assitant` (the directory is `wled-assistant`).
- `README.md` holds user-facing docs (flashing, HyperHDR, Home Assistant, test checklist, limitations). Keep it in sync with behaviour changes.
- LED output (direct 3.3V GPIO drive at 4.75V PSU) is hardware-confirmed as of 2026-10-08. The BME280 is not yet on hardware (replaces an HTU21D whose humidity channel died after solvent cleaning, 2026-10-29); MQTT/HA and full HyperHDR streaming are still unverified on hardware; claims about them come from source analysis.
