"""CAN TX/RX worker shared by the host GUI."""

from __future__ import annotations

import queue
import threading
import time

import can

import protocol as proto

_RX_SLICE_S = 0.001


class CanWorker(threading.Thread):
    def __init__(self, bus: can.BusABC, node_id: int, rx_q: queue.Queue) -> None:
        super().__init__(daemon=True)
        self.bus = bus
        self.ids = proto.ids(node_id)
        self.rx_q = rx_q
        self.tx_q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._cyclic = False
        self._period = 0.005
        self._cyclic_id = 0
        self._payload_fn = lambda: proto.pack_motion(0.0, 0.0, 0.0)
        self._epoch = 0
        self._lock = threading.Lock()

    def set_cyclic(self, enabled: bool, rate_hz: float, can_id: int, payload_fn) -> None:
        with self._lock:
            self._cyclic = enabled
            self._period = 1.0 / max(1.0, float(rate_hz))
            self._cyclic_id = int(can_id)
            self._payload_fn = payload_fn
            self._epoch += 1

    def is_cyclic(self) -> bool:
        with self._lock:
            return self._cyclic

    def send(self, can_id: int, data: bytes) -> None:
        self.tx_q.put((can_id, data))

    def stop(self) -> None:
        self._stop.set()

    def _send_frame(self, can_id: int, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))

    def _handle_rx(self, msg: can.Message) -> None:
        if msg.is_extended_id or len(msg.data) != 8:
            return
        aid = msg.arbitration_id
        data = bytes(msg.data)
        if aid == self.ids["fb"]:
            self.rx_q.put(("fb", proto.unpack_feedback(data)))
        elif aid == self.ids["ack"]:
            self.rx_q.put(("ack", proto.unpack_ack(data)))
        elif aid == self.ids["status"]:
            self.rx_q.put(("status", proto.unpack_status(data)))
        elif aid == self.ids["cali"]:
            self.rx_q.put(("cali", proto.unpack_cali(data)))

    def run(self) -> None:
        next_tx = 0.0
        seen_epoch = -1
        while not self._stop.is_set():
            try:
                while True:
                    can_id, data = self.tx_q.get_nowait()
                    self._send_frame(can_id, data)
            except queue.Empty:
                pass
            except Exception as exc:
                self.rx_q.put(("error", str(exc)))

            now = time.monotonic()
            with self._lock:
                cyclic = self._cyclic
                period = self._period
                cyclic_id = self._cyclic_id
                payload_fn = self._payload_fn
                epoch = self._epoch

            if cyclic:
                if epoch != seen_epoch:
                    next_tx = now
                    seen_epoch = epoch
                if now >= next_tx:
                    try:
                        self._send_frame(cyclic_id, payload_fn())
                    except Exception as exc:
                        self.rx_q.put(("error", str(exc)))
                    next_tx += period
                    if next_tx < now:
                        next_tx = now + period
            else:
                seen_epoch = epoch

            leftover = (next_tx - time.monotonic()) if cyclic else _RX_SLICE_S
            timeout = min(_RX_SLICE_S, max(0.0, leftover))
            try:
                msg = self.bus.recv(timeout=timeout)
            except Exception as exc:
                self.rx_q.put(("error", str(exc)))
                continue
            while msg is not None:
                self._handle_rx(msg)
                try:
                    msg = self.bus.recv(timeout=0.0)
                except Exception as exc:
                    self.rx_q.put(("error", str(exc)))
                    break
