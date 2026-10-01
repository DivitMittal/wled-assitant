---
description: tools/ knowledge — host-side Adalight test sender and USB-Serial/JTAG caveats
applyTo: "tools/**"
---

# AGENTS (tools/)

## OVERVIEW

`adalight_test.py` (pyserial, run with `uv run --with pyserial`) streams Adalight frames, probes WLED with `v`, and injects malformed frames to test recovery.

## FACTS

- Frame: `'A' 'd' 'a' (n-1)>>8 (n-1)&0xFF hi^lo^0x55` + n×RGB; 144 LEDs = 438 B; 60 fps = 26.3 kB/s. Baud is ignored by USB CDC.
- WLED parses channel order R,G,B; the strip's GRB order is applied by WLED's LED Preferences, not by the sender.
- In its idle state WLED's parser treats `{`, `v`, `I`, `l`, `L`, `o`, `O`, `0xB0–0xB7`, `0xC9` as commands. Injected noise must avoid these unless testing desync on purpose.

## ANTI-PATTERNS

- Never toggle DTR/RTS (e.g. `ser.dtr = False` before open): RTS asserted with DTR low **resets** the ESP32-S3 via USB-Serial/JTAG. Keep pyserial defaults.
- Don't hold the port while flashing or while HyperHDR is running.
