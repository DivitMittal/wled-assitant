"""Unit tests for the Adalight frame encoder in tools/adalight_test.py.

The header layout must match what WLED's wled00/wled_serial.cpp accepts; a wrong checksum makes
WLED silently ignore every frame, which looks like a wiring fault on the bench.
"""
import importlib.util
import pathlib
import sys
import types

import pytest

TOOL = pathlib.Path(__file__).resolve().parent.parent / "tools" / "adalight_test.py"


@pytest.fixture(scope="module")
def ada():
    # The encoder never touches the port, so a stub keeps these tests independent of pyserial.
    sys.modules.setdefault("serial", types.ModuleType("serial"))
    spec = importlib.util.spec_from_file_location("adalight_test", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("n", [1, 144, 256, 300, 65536])
def test_header_layout_and_checksum(ada, n):
    h = ada.ada_header(n)
    assert len(h) == 6
    assert h[:3] == b"Ada"
    count = n - 1
    assert (h[3], h[4]) == (count >> 8, count & 0xFF)
    assert h[5] == h[3] ^ h[4] ^ 0x55


def test_header_for_144_leds_matches_known_bytes(ada):
    # 143 = 0x008F -> checksum 0x00 ^ 0x8F ^ 0x55 = 0xDA
    assert ada.ada_header(144) == bytes((0x41, 0x64, 0x61, 0x00, 0x8F, 0xDA))


@pytest.mark.parametrize("n", [1, 144, 300])
def test_rainbow_payload_size_and_range(ada, n):
    payload = ada.rainbow(n, 0.0)
    assert len(payload) == 3 * n
    # value 0.5 caps every channel at half brightness, keeping bench tests within PSU budget
    assert max(payload) <= 127


def test_rainbow_advances_with_time(ada):
    assert ada.rainbow(144, 0.0) != ada.rainbow(144, 1.0)


def test_full_frame_fits_default_rx_buffer(ada):
    # 144 LEDs is the default strip; WLEDA_USB_RX_BUFFER must hold at least one whole frame
    assert len(ada.ada_header(144)) + len(ada.rainbow(144, 0.0)) == 438
