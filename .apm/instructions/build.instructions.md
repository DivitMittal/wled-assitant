---
description: Build/flash knowledge — platformio_override.ini, bootstrap/build scripts, ESP32-S3 USB flags
applyTo: "{platformio_override.ini,scripts/**}"
---

# AGENTS (build: platformio_override.ini, scripts/)

## OVERVIEW

`platformio_override.ini` is symlinked to `wled/platformio_override.ini` (WLED's supported override hook) by `scripts/bootstrap.sh`. Edit the root copy only.

## RULES

- `[wled_assitant]` section is the **only** place pins, LED count, and the ABL default are written; `pin_flags` derives `DATA_PINS`, `PIXEL_COUNTS`, `I2C*PIN`, `WLEDA_*`. Never duplicate literals elsewhere.
- Keep `ARDUINO_USB_MODE=1` (USB-Serial/JTAG, hardware-backed, crash-proof flashing) and `ARDUINO_USB_CDC_ON_BOOT=1` (`Serial` = USB). `USB_MODE=0` (TinyUSB) is not supported by the usermod (`#error`).
- Usermod is linked via `custom_usermods = wled-assitant = symlink://../usermods/wled_assitant` (path relative to `wled/`). This replaces upstream's `audioreactive` default on purpose.
- `bootstrap.sh` must keep refusing a modified WLED core; compare with `git diff --ignore-cr-at-eol` (upstream ships CRLF files that otherwise look modified).
- Board: upstream `esp32s3_4M_qspi` / `lolin_s3_mini` (same S3FH4R2 chip). Partition: `WLED_ESP32_4MB_1MB_FS.csv` (2 × 1.5 MB OTA). Check the `Flash:` line after every build.

## VERIFYING A CHANGE

```bash
scripts/build.sh                           # must end in SUCCESS, no warnings from wled_assitant.cpp
grep -E "usermod object entries|Flash:" <build log>
```

Negative pin tests: compile the usermod TU (from `pio run -t compiledb`) with `-UWLEDA_TXS_OE_PIN -DWLEDA_TXS_OE_PIN=20` etc. appended — must hit a `static_assert`.

## DEVSHELL (flake/devshells.nix)

- Tools: `platformio` (FHS-wrapped on Linux), `esptool` v5 (binary is `esptool`, subcommands `flash-id`/`write-flash`; no `esptool.py`), `nodejs`, `tio`, `uv`, `python` + pyserial, `clang-tools`, `mosquitto`, `apm`.
- Commands (category `firmware`): `fw-build`, `fw-flash <port>`, `fw-erase <port>`, `fw-info <port>`, `fw-monitor <port>`, `fw-ports`, `fw-compiledb`. Port-taking commands fail fast without an argument.
- This Mac is x86_64-darwin → packages come from `nixpkgs-2605` (unstable dropped x86_64-darwin). Check new packages with `nix eval --inputs-from . nixpkgs-2605#<pkg>.version`.
- Known harmless noise with nix PlatformIO: `Installing Python dependencies … error: externally-managed-environment`. The Tasmota platform tries to pip-install `wheel`, `zopfli` and `tasmota-metrics` into the read-only nix Python. Neither the platform builder nor WLED uses them; the build and upload paths (bundled esptool 4.7.4 + pyserial) were verified to work. Only bundled `espsecure`/`espefuse` lack `cryptography`; use the devshell's `esptool` v5 tools if ever needed.

## FLASHING

Stop HyperHDR's LED device first (single port owner). `pio run -e wled_assitant_s3_supermini -t upload --upload-port /dev/cu.usbmodemXXXX`; first time add `-t erase`. Recovery: hold BOOT, tap RST.
