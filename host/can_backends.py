"""Platform-aware python-can interface / channel helpers."""

from __future__ import annotations

import glob
import sys
from typing import Any

import can

_WIN = sys.platform == "win32"
_LINUX = sys.platform.startswith("linux")
_MAC = sys.platform == "darwin"

# Shown in the GUI; extras can still be typed in.
INTERFACES: tuple[str, ...] = (
    ("pcan", "slcan", "usb2can", "kvaser", "vector", "virtual")
    if _WIN
    else ("socketcan", "pcan", "slcan", "usb2can", "kvaser", "virtual")
    if _LINUX
    else ("slcan", "pcan", "usb2can", "virtual")
)

BITRATES: tuple[int, ...] = (1_000_000, 500_000, 250_000, 125_000, 100_000)


def default_interface() -> str:
    if _WIN:
        return "pcan"
    if _LINUX:
        return "socketcan"
    return "slcan"


def default_channel(interface: str | None = None) -> str:
    iface = (interface or default_interface()).strip().lower()
    presets = _preset_channels(iface)
    detected = detect_channels(iface)
    if detected:
        return detected[0]
    return presets[0] if presets else ""


def _preset_channels(interface: str) -> list[str]:
    iface = interface.strip().lower()
    if iface == "pcan":
        names = [f"PCAN_USBBUS{i}" for i in range(1, 9)]
        if not _WIN:
            names += [f"PCAN_PCIBUS{i}" for i in range(1, 3)]
        return names
    if iface == "socketcan":
        return [f"can{i}" for i in range(4)] + ["vcan0"]
    if iface == "slcan":
        return _serial_ports()
    if iface == "virtual":
        return ["vcan0", "test"]
    if iface in ("usb2can", "kvaser", "vector", "ixxat", "nican"):
        return ["0", "1"]
    return []


def _serial_ports() -> list[str]:
    ports: list[str] = []
    try:
        from serial.tools import list_ports

        ports = [p.device for p in list_ports.comports() if p.device]
    except Exception:
        if _WIN:
            ports = [f"COM{i}" for i in range(1, 21)]
        elif _MAC:
            ports = sorted(glob.glob("/dev/cu.usb*") + glob.glob("/dev/tty.usb*"))
        else:
            ports = sorted(
                glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyS*")
            )
    if not ports:
        if _WIN:
            ports = ["COM3", "COM4"]
        elif _MAC:
            ports = ["/dev/cu.usbserial", "/dev/cu.usbmodem"]
        else:
            ports = ["/dev/ttyACM0", "/dev/ttyUSB0"]
    return ports


def detect_channels(interface: str) -> list[str]:
    iface = interface.strip().lower()
    found: list[str] = []
    try:
        for cfg in can.detect_available_configs(interfaces=[iface]):
            ch = cfg.get("channel")
            if ch is None:
                continue
            text = str(ch)
            if text and text not in found:
                found.append(text)
    except Exception:
        pass
    if iface == "slcan":
        for port in _serial_ports():
            if port not in found:
                found.append(port)
    return found


def list_channels(interface: str) -> list[str]:
    iface = interface.strip().lower()
    detected = detect_channels(iface)
    presets = _preset_channels(iface)
    out: list[str] = []
    for ch in detected + presets:
        if ch and ch not in out:
            out.append(ch)
    return out


def open_bus(interface: str, channel: str, bitrate: int) -> can.BusABC:
    iface = interface.strip()
    ch = channel.strip()
    kwargs: dict[str, Any] = {
        "interface": iface,
        "channel": _typed_channel(iface, ch),
        "bitrate": int(bitrate),
    }
    if iface.lower() == "socketcan":
        kwargs.pop("bitrate", None)
    return can.Bus(**kwargs)


def _typed_channel(interface: str, channel: str) -> str | int:
    if interface.strip().lower() in ("kvaser", "vector", "ixxat", "nican"):
        try:
            return int(channel)
        except ValueError:
            return channel
    return channel
