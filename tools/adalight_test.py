#!/usr/bin/env python3
"""Drive the WLED USB Adalight path without HyperHDR (acceptance tests).

  uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodem1101 --probe
  uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodem1101 --fps 60 --seconds 20
  uv run --with pyserial tools/adalight_test.py /dev/cu.usbmodem1101 --malformed --seconds 10

Frame format (what HyperHDR's "Ada" protocol sends, and what wled00/wled_serial.cpp parses):
  'A' 'd' 'a' <(n-1) hi> <(n-1) lo> <hi ^ lo ^ 0x55> then n * (R, G, B)

Leave DTR/RTS at their defaults: on the ESP32-S3 USB-Serial/JTAG port, dropping DTR while RTS is
asserted resets the chip (that is how esptool resets it).
"""
import argparse
import colorsys
import time

import serial


def ada_header(n: int) -> bytes:
    hi, lo = (n - 1) >> 8, (n - 1) & 0xFF
    return bytes((ord("A"), ord("d"), ord("a"), hi, lo, hi ^ lo ^ 0x55))


def rainbow(n: int, t: float) -> bytes:
    out = bytearray()
    for i in range(n):
        r, g, b = colorsys.hsv_to_rgb((i / n + t * 0.2) % 1.0, 1.0, 0.5)
        out += bytes((int(r * 255), int(g * 255), int(b * 255)))
    return bytes(out)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--leds", type=int, default=144)
    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--probe", action="store_true", help="ask WLED for its version over the same port and exit")
    ap.add_argument("--malformed", action="store_true", help="every 10th frame is truncated, followed by noise")
    args = ap.parse_args()

    # Baud is ignored by USB CDC; any value works.
    with serial.Serial(args.port, 115200, timeout=0.5) as port:
        if args.probe:
            port.reset_input_buffer()
            port.write(b"v")
            time.sleep(0.3)
            print(port.read(256).decode(errors="replace").strip() or "no reply (is serial RX enabled in WLED?)")
            return

        header = ada_header(args.leds)
        frame_len = len(header) + 3 * args.leds
        print(f"{args.leds} LEDs -> {frame_len} B/frame -> {frame_len * args.fps / 1000:.1f} kB/s at {args.fps:g} fps")

        period = 1.0 / args.fps
        start = next_at = time.monotonic()
        frames = sent_bytes = 0
        while time.monotonic() - start < args.seconds:
            payload = header + rainbow(args.leds, time.monotonic() - start)
            if args.malformed and frames % 10 == 9:
                # half a frame, then bytes that are not an Adalight header ('A' never appears)
                payload = payload[: frame_len // 2] + bytes(range(0x80, 0xA0))
            sent_bytes += port.write(payload)
            frames += 1
            next_at += period
            time.sleep(max(0.0, next_at - time.monotonic()))
        port.flush()
        elapsed = time.monotonic() - start
        print(f"sent {frames} frames in {elapsed:.1f}s = {frames / elapsed:.1f} fps, {sent_bytes / elapsed / 1000:.1f} kB/s")
        print("stream stopped: WLED should return to its normal effect after the realtime timeout (2.5 s default)")


if __name__ == "__main__":
    main()
