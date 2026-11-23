"""Static checks on platformio_override.ini, the single source of truth for the wiring.

The usermod's static_asserts only run inside a full PlatformIO build; these run in seconds and
catch the same class of mistake (pin collisions, pins wired to the USB/flash bus, an RX queue too
small for one Adalight frame) before CI spends minutes on a firmware build.
"""
import configparser
import pathlib
import re

import pytest

INI = pathlib.Path(__file__).resolve().parent.parent / "platformio_override.ini"

# Pins that can never carry a peripheral on the ESP32-S3FH4R2 build: USB D-/D+, the in-package
# flash/PSRAM bus, and UART0 RX (allocating it makes WLED disable Adalight serial RX).
HARD_RESERVED = {19, 20, *range(22, 33), 44}
ADALIGHT_HEADER = 6


def _ini():
    cp = configparser.ConfigParser(inline_comment_prefixes=(";", "#"), interpolation=None, strict=False)
    cp.read(INI)
    return cp


@pytest.fixture(scope="module")
def ini():
    return _ini()


@pytest.fixture(scope="module")
def pins(ini):
    section = ini["wled_assitant"]
    found = {k: int(v) for k, v in section.items() if k.endswith("_pin")}
    assert found, "no *_pin keys in [wled_assitant]"
    return found


def test_pins_are_distinct(pins):
    by_gpio = {}
    for name, gpio in pins.items():
        by_gpio.setdefault(gpio, []).append(name)
    clashes = {g: n for g, n in by_gpio.items() if len(n) > 1}
    assert not clashes, f"GPIOs assigned twice: {clashes}"


def test_pins_avoid_hard_reserved(pins):
    bad = {n: g for n, g in pins.items() if g in HARD_RESERVED or not 0 <= g <= 48}
    assert not bad, f"pins on reserved/nonexistent GPIOs: {bad}"


def test_rx_buffer_holds_one_frame(ini):
    leds = int(ini["wled_assitant"]["led_count"])
    flags = ini["env:wled_assitant_s3_supermini"]["build_flags"]
    m = re.search(r"-D\s*WLEDA_USB_RX_BUFFER=(\d+)", flags)
    rx = int(m.group(1)) if m else 2048  # usermod default
    assert rx >= ADALIGHT_HEADER + 3 * leds, f"RX buffer {rx} B < one {leds}-LED frame"


def test_pin_flags_come_from_the_wiring_section(ini):
    # Hard-coded numbers in build_flags would silently diverge from [wled_assitant].
    flags = ini["wled_assitant"]["pin_flags"]
    for define in re.findall(r"-D\s*(\w+)=(\S+)", flags):
        assert define[1].startswith("${wled_assitant."), f"{define[0]} is hard-coded: {define[1]}"
