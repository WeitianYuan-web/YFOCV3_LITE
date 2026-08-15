#!/usr/bin/env python3
"""Minimal YFOCV3 host: send MIT-style control frames and parse feedback."""

from __future__ import annotations

import argparse
import math
import sys
import time

try:
    import can
except ImportError:
    print("python-can is required: pip install python-can", file=sys.stderr)
    raise

PI = math.pi
POS_CMD = (-4.0 * PI, 4.0 * PI)
VEL = (-100.0, 100.0)
KP = (0.0, 500.0)
KD = (0.0, 5.0)
POS_FB = (-4.0 * PI, 4.0 * PI)
TORQUE = (-1.0, 1.0)


def encode_u16(value: float, min_v: float, max_v: float) -> int:
    span = max_v - min_v
    t = 0.0 if span == 0.0 else (value - min_v) / span
    t = min(max(t, 0.0), 1.0)
    return int(t * 65535.0 + 0.5)


def decode_u16(raw: int, min_v: float, max_v: float) -> float:
    return min_v + (raw * (max_v - min_v) / 65535.0)


def pack_control(pos: float, vel: float, kp: float, kd: float) -> bytes:
    return b"".join(
        encode_u16(v, lo, hi).to_bytes(2, "big")
        for v, (lo, hi) in (
            (pos, POS_CMD),
            (vel, VEL),
            (kp, KP),
            (kd, KD),
        )
    )


def unpack_feedback(data: bytes) -> dict[str, float | int]:
    if len(data) != 8:
        raise ValueError("feedback DLC must be 8")
    pos = decode_u16(int.from_bytes(data[0:2], "big"), *POS_FB)
    vel = decode_u16(int.from_bytes(data[2:4], "big"), *VEL)
    torque = decode_u16(int.from_bytes(data[4:6], "big"), *TORQUE)
    turns = int.from_bytes(data[6:8], "big")
    return {"pos": pos, "vel": vel, "torque": torque, "turns": turns}


def parse_args() -> argparse.Namespace:
    default_interface = "pcan" if sys.platform == "win32" else "socketcan"
    default_channel = "PCAN_USBBUS1" if sys.platform == "win32" else "can0"
    parser = argparse.ArgumentParser(description="YFOCV3 voltage servo host")
    parser.add_argument("--interface", default=default_interface, help="python-can interface")
    parser.add_argument("--channel", default=default_channel, help="CAN channel / slcan device")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--id", type=int, default=1, help="device node id (nibble)")
    parser.add_argument("--pos", type=float, default=0.0, help="target angle rad")
    parser.add_argument("--vel", type=float, default=0.0, help="target velocity rad/s")
    parser.add_argument("--kp", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=200.0, help="control TX rate Hz")
    parser.add_argument("--listen", action="store_true", help="only parse feedback")
    parser.add_argument("--once", action="store_true", help="send one control frame and exit after first feedback")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    node_id = args.id & 0xF
    cmd_id = 0x100 + node_id
    fb_id = 0x200 + node_id

    bus = can.Bus(interface=args.interface, channel=args.channel, bitrate=args.bitrate)
    payload = pack_control(args.pos, args.vel, args.kp, args.kd)
    period = 0.0 if args.rate <= 0.0 else 1.0 / args.rate
    last_tx = 0.0

    print(f"cmd=0x{cmd_id:03X} fb=0x{fb_id:03X} pos={args.pos:.3f} vel={args.vel:.3f} kp={args.kp:.3f} kd={args.kd:.3f}")
    try:
        while True:
            now = time.monotonic()
            if not args.listen and (last_tx == 0.0 or (now - last_tx) >= period):
                bus.send(can.Message(arbitration_id=cmd_id, data=payload, is_extended_id=False))
                last_tx = now

            msg = bus.recv(timeout=0.05)
            if msg is None:
                continue
            if msg.arbitration_id != fb_id or msg.is_extended_id:
                continue
            fb = unpack_feedback(bytes(msg.data))
            print(
                f"fb pos={fb['pos']:+.4f} rad  vel={fb['vel']:+.3f} rad/s  "
                f"t={fb['torque']:+.3f}  turns={fb['turns']}"
            )
            if args.once:
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        bus.shutdown()


if __name__ == "__main__":
    sys.exit(main())
