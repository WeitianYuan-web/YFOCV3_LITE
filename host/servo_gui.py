#!/usr/bin/env python3
"""YFOCV3 GUI host for Motor CAN Protocol V1.0."""

from __future__ import annotations

import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import can
except ImportError:
    print("python-can is required: pip install python-can", file=sys.stderr)
    raise

import protocol as proto


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
        self._motion_fn = lambda: proto.pack_motion(0.0, 0.0, 0.0)
        self._lock = threading.Lock()

    def set_cyclic(self, enabled: bool, rate_hz: float, motion_fn) -> None:
        with self._lock:
            self._cyclic = enabled
            self._period = 1.0 / max(1.0, float(rate_hz))
            self._motion_fn = motion_fn

    def send(self, can_id: int, data: bytes) -> None:
        self.tx_q.put((can_id, data))

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        last_tx = 0.0
        while not self._stop.is_set():
            try:
                while True:
                    can_id, data = self.tx_q.get_nowait()
                    self.bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False))
            except queue.Empty:
                pass
            except Exception as exc:
                self.rx_q.put(("error", str(exc)))

            now = time.monotonic()
            with self._lock:
                cyclic = self._cyclic
                period = self._period
                motion_fn = self._motion_fn
            if cyclic and (now - last_tx) >= period:
                try:
                    self.bus.send(
                        can.Message(
                            arbitration_id=self.ids["motion"],
                            data=motion_fn(),
                            is_extended_id=False,
                        )
                    )
                    last_tx = now
                except Exception as exc:
                    self.rx_q.put(("error", str(exc)))

            try:
                msg = self.bus.recv(timeout=0.005)
            except Exception as exc:
                self.rx_q.put(("error", str(exc)))
                continue
            if msg is None or msg.is_extended_id or len(msg.data) != 8:
                continue
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


class HostGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("YFOCV3 Motor Host")
        self.bus: can.BusABC | None = None
        self.worker: CanWorker | None = None
        self.rx_q: queue.Queue = queue.Queue()
        self.seq = 1
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self._snap = (0.0, 0.0, 0.0)
        self._snap_lock = threading.Lock()
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(30, self._poll_rx)

    def _build(self) -> None:
        pad = {"padx": 6, "pady": 4}
        conn = ttk.LabelFrame(self.root, text="连接")
        conn.pack(fill="x", **pad)
        self.var_iface = tk.StringVar(value="pcan" if sys.platform == "win32" else "socketcan")
        self.var_ch = tk.StringVar(value="PCAN_USBBUS1" if sys.platform == "win32" else "can0")
        self.var_bitrate = tk.StringVar(value="1000000")
        self.var_id = tk.StringVar(value="1")
        self.lbl_conn = ttk.Label(conn, text="未连接", foreground="gray")
        ttk.Label(conn, text="接口").grid(row=0, column=0, sticky="e")
        ttk.Entry(conn, textvariable=self.var_iface, width=12).grid(row=0, column=1)
        ttk.Label(conn, text="通道").grid(row=0, column=2, sticky="e")
        ttk.Entry(conn, textvariable=self.var_ch, width=16).grid(row=0, column=3)
        ttk.Label(conn, text="波特率").grid(row=0, column=4, sticky="e")
        ttk.Entry(conn, textvariable=self.var_bitrate, width=10).grid(row=0, column=5)
        ttk.Label(conn, text="Motor ID").grid(row=0, column=6, sticky="e")
        ttk.Entry(conn, textvariable=self.var_id, width=6).grid(row=0, column=7)
        ttk.Button(conn, text="连接", command=self.connect).grid(row=0, column=8, padx=4)
        ttk.Button(conn, text="断开", command=self.disconnect).grid(row=0, column=9)
        self.lbl_conn.grid(row=0, column=10, padx=8)

        fb = ttk.LabelFrame(self.root, text="Motion Feedback / Status")
        fb.pack(fill="x", **pad)
        self.var_fb_pos = tk.StringVar(value="—")
        self.var_fb_vel = tk.StringVar(value="—")
        self.var_fb_t = tk.StringVar(value="—")
        self.var_fb_hz = tk.StringVar(value="0 Hz")
        self.var_state = tk.StringVar(value="—")
        self.var_fault = tk.StringVar(value="—")
        self.var_mode = tk.StringVar(value="—")
        big = ("Segoe UI", 16, "bold")
        ttk.Label(fb, text="位置 rad").grid(row=0, column=0)
        ttk.Label(fb, textvariable=self.var_fb_pos, font=big).grid(row=1, column=0, padx=12)
        ttk.Label(fb, text="速度 rad/s").grid(row=0, column=1)
        ttk.Label(fb, textvariable=self.var_fb_vel, font=big).grid(row=1, column=1, padx=12)
        ttk.Label(fb, text="电压 pu").grid(row=0, column=2)
        ttk.Label(fb, textvariable=self.var_fb_t, font=big).grid(row=1, column=2, padx=12)
        ttk.Label(fb, text="反馈频率").grid(row=0, column=3)
        ttk.Label(fb, textvariable=self.var_fb_hz).grid(row=1, column=3, padx=12)
        ttk.Label(fb, text="State").grid(row=0, column=4)
        ttk.Label(fb, textvariable=self.var_state).grid(row=1, column=4, padx=8)
        ttk.Label(fb, text="Fault").grid(row=0, column=5)
        ttk.Label(fb, textvariable=self.var_fault).grid(row=1, column=5, padx=8)
        ttk.Label(fb, text="Mode").grid(row=0, column=6)
        ttk.Label(fb, textvariable=self.var_mode).grid(row=1, column=6, padx=8)

        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, **pad)
        self._tab_motion(nb)
        self._tab_vel(nb)
        self._tab_pos(nb)
        self._tab_gains(nb)
        self._tab_mgmt(nb)

        logf = ttk.LabelFrame(self.root, text="日志")
        logf.pack(fill="both", expand=True, **pad)
        self.log = tk.Text(logf, height=10, wrap="word")
        scroll = ttk.Scrollbar(logf, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _entry_row(self, parent, row: int, label: str, var: tk.StringVar, width: int = 12) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e", padx=4, pady=3)
        ttk.Entry(parent, textvariable=var, width=width).grid(row=row, column=1, sticky="w", padx=4, pady=3)

    def _tab_motion(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text="Motion")
        self.var_m_pos = tk.StringVar(value="0.0")
        self.var_m_vel = tk.StringVar(value="0.0")
        self.var_m_ff = tk.StringVar(value="0.0")
        self.var_m_rate = tk.StringVar(value="200")
        self._entry_row(tab, 0, "位置 rad", self.var_m_pos)
        self._entry_row(tab, 1, "速度 rad/s", self.var_m_vel)
        self._entry_row(tab, 2, "Voltage FF 0~1", self.var_m_ff)
        self._entry_row(tab, 3, "循环频率 Hz", self.var_m_rate)
        btns = ttk.Frame(tab)
        btns.grid(row=4, column=0, columnspan=3, pady=8, sticky="w")
        ttk.Button(btns, text="发送一次", command=self.send_motion_once).pack(side="left", padx=4)
        ttk.Button(btns, text="开始循环", command=self.start_cyclic).pack(side="left", padx=4)
        ttk.Button(btns, text="停止循环", command=self.stop_cyclic).pack(side="left", padx=4)
        ttk.Button(btns, text="用反馈位置填入", command=self.fill_pos_from_fb).pack(side="left", padx=4)
        ttk.Label(tab, text="循环发送时会按当前输入框的值更新，并刷新上方反馈。").grid(
            row=5, column=0, columnspan=3, sticky="w", padx=4
        )

    def _tab_vel(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text="Velocity")
        self.var_v_vel = tk.StringVar(value="0.0")
        self._entry_row(tab, 0, "目标速度 rad/s", self.var_v_vel)
        ttk.Button(tab, text="发送 Velocity Command", command=self.send_velocity).grid(
            row=1, column=0, columnspan=2, pady=8, sticky="w", padx=4
        )
        ttk.Label(tab, text="当前固件未实现速度模式，驱动器会忽略该帧。").grid(
            row=2, column=0, columnspan=2, sticky="w", padx=4
        )

    def _tab_pos(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text="Position")
        self.var_p_pos = tk.StringVar(value="0.0")
        self.var_p_maxv = tk.StringVar(value="10.0")
        self._entry_row(tab, 0, "目标位置 rad", self.var_p_pos)
        self._entry_row(tab, 1, "最大速度 rad/s", self.var_p_maxv)
        ttk.Button(tab, text="发送 Position Command", command=self.send_position).grid(
            row=2, column=0, columnspan=2, pady=8, sticky="w", padx=4
        )
        ttk.Label(tab, text="当前固件未实现位置模式，驱动器会忽略该帧。").grid(
            row=3, column=0, columnspan=2, sticky="w", padx=4
        )

    def _tab_gains(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text="Gains")
        self.var_g_mode = tk.StringVar(value="MOTION")
        self.var_g_kp = tk.StringVar(value="0.0")
        self.var_g_ki = tk.StringVar(value="0.0")
        self.var_g_kd = tk.StringVar(value="0.0")
        ttk.Label(tab, text="参数组").grid(row=0, column=0, sticky="e", padx=4, pady=3)
        ttk.Combobox(
            tab,
            textvariable=self.var_g_mode,
            values=("MOTION", "VELOCITY", "POSITION"),
            state="readonly",
            width=12,
        ).grid(row=0, column=1, sticky="w")
        self._entry_row(tab, 1, "Kp", self.var_g_kp)
        self._entry_row(tab, 2, "Ki", self.var_g_ki)
        self._entry_row(tab, 3, "Kd", self.var_g_kd)
        ttk.Button(tab, text="发送 SET_GAINS", command=self.send_gains).grid(
            row=4, column=0, columnspan=2, pady=8, sticky="w", padx=4
        )
        ttk.Label(
            tab,
            text="MOTION：Kp/Kd，Ki 必须为 0（当前固件只接受这一组）。\n"
            "VELOCITY：Kp/Ki，Kd=0。POSITION：Kp/Ki/Kd。",
        ).grid(row=5, column=0, columnspan=2, sticky="w", padx=4)

    def _tab_mgmt(self, nb: ttk.Notebook) -> None:
        tab = ttk.Frame(nb)
        nb.add(tab, text="Management")
        self.var_set_mode = tk.StringVar(value="MOTION")
        row = ttk.Frame(tab)
        row.grid(row=0, column=0, sticky="w", pady=6)
        for text, cmd in (
            ("ENABLE", proto.CMD_ENABLE),
            ("DISABLE", proto.CMD_DISABLE),
            ("SET_ZERO", proto.CMD_SET_ZERO),
            ("CLEAR_FAULT", proto.CMD_CLEAR_FAULT),
            ("START_CALI", proto.CMD_START_CALI),
            ("GET_STATUS", proto.CMD_GET_STATUS),
        ):
            ttk.Button(row, text=text, command=lambda c=cmd: self.send_mgmt(c)).pack(side="left", padx=3)
        mode_row = ttk.Frame(tab)
        mode_row.grid(row=1, column=0, sticky="w", pady=6)
        ttk.Label(mode_row, text="SET_CONTROL_MODE").pack(side="left", padx=4)
        ttk.Combobox(
            mode_row,
            textvariable=self.var_set_mode,
            values=("MOTION", "VELOCITY", "POSITION"),
            state="readonly",
            width=12,
        ).pack(side="left")
        ttk.Button(mode_row, text="发送", command=self.send_set_mode).pack(side="left", padx=6)
        ttk.Label(
            tab,
            text="当前固件：SET_ZERO / GET_STATUS / SET_CONTROL_MODE(MOTION) 可用。\n"
            "ENABLE/DISABLE/校准/速度位置模式会返回 INVALID_COMMAND 或被忽略。",
        ).grid(row=2, column=0, sticky="w", padx=4, pady=8)

    def _log(self, text: str) -> None:
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def _need_worker(self) -> CanWorker | None:
        if self.worker is None:
            messagebox.showwarning("未连接", "请先连接 CAN")
            return None
        return self.worker

    def _next_seq(self) -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def _f(self, var: tk.StringVar) -> float:
        return float(var.get().strip())

    def connect(self) -> None:
        self.disconnect()
        try:
            node_id = int(self.var_id.get())
            bus = can.Bus(
                interface=self.var_iface.get().strip(),
                channel=self.var_ch.get().strip(),
                bitrate=int(self.var_bitrate.get()),
            )
        except Exception as exc:
            messagebox.showerror("连接失败", str(exc))
            return
        self.bus = bus
        self.rx_q = queue.Queue()
        self.worker = CanWorker(bus, node_id, self.rx_q)
        self.worker.start()
        self.lbl_conn.configure(text=f"已连接  ID={node_id}", foreground="green")
        self._log(f"connected iface={self.var_iface.get()} ch={self.var_ch.get()} id={node_id}")

    def disconnect(self) -> None:
        if self.worker is not None:
            self.worker.set_cyclic(False, 1.0, lambda: proto.pack_motion(0.0, 0.0, 0.0))
            self.worker.stop()
            self.worker.join(timeout=0.5)
            self.worker = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.lbl_conn.configure(text="未连接", foreground="gray")

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()

    def send_motion_once(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = self._refresh_snap()
        except ValueError:
            messagebox.showerror("参数错误", "位置/速度/FF 必须是数字")
            return
        w.send(w.ids["motion"], data)
        self._log(
            f"TX Motion pos={self.var_m_pos.get()} vel={self.var_m_vel.get()} ff={self.var_m_ff.get()}"
        )

    def _refresh_snap(self) -> bytes:
        pos = self._f(self.var_m_pos)
        vel = self._f(self.var_m_vel)
        ff = self._f(self.var_m_ff)
        with self._snap_lock:
            self._snap = (pos, vel, ff)
        return proto.pack_motion(pos, vel, ff)

    def _motion_payload(self) -> bytes:
        with self._snap_lock:
            pos, vel, ff = self._snap
        return proto.pack_motion(pos, vel, ff)

    def start_cyclic(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            rate = float(self.var_m_rate.get())
            self._refresh_snap()
        except ValueError:
            messagebox.showerror("参数错误", "频率和 Motion 参数必须是数字")
            return
        w.set_cyclic(True, rate, self._motion_payload)
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self._log(f"cyclic Motion {rate:.1f} Hz")

    def stop_cyclic(self) -> None:
        if self.worker is not None:
            self.worker.set_cyclic(False, 1.0, self._motion_payload)
        self._log("cyclic Motion stopped")

    def fill_pos_from_fb(self) -> None:
        text = self.var_fb_pos.get()
        if text not in ("—", ""):
            self.var_m_pos.set(text)

    def send_velocity(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = proto.pack_velocity(self._f(self.var_v_vel))
        except ValueError:
            messagebox.showerror("参数错误", "速度必须是数字")
            return
        w.send(w.ids["vel"], data)
        self._log(f"TX Velocity vel={self.var_v_vel.get()}")

    def send_position(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = proto.pack_position(self._f(self.var_p_pos), self._f(self.var_p_maxv))
        except ValueError:
            messagebox.showerror("参数错误", "位置/最大速度必须是数字")
            return
        w.send(w.ids["pos"], data)
        self._log(f"TX Position pos={self.var_p_pos.get()} max_vel={self.var_p_maxv.get()}")

    def send_gains(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        mode_map = {"MOTION": proto.MODE_MOTION, "VELOCITY": proto.MODE_VELOCITY, "POSITION": proto.MODE_POSITION}
        mode = mode_map[self.var_g_mode.get()]
        try:
            kp, ki, kd = self._f(self.var_g_kp), self._f(self.var_g_ki), self._f(self.var_g_kd)
        except ValueError:
            messagebox.showerror("参数错误", "Kp/Ki/Kd 必须是数字")
            return
        seq = self._next_seq()
        w.send(w.ids["gains"], proto.pack_gains(mode, seq, kp, ki, kd))
        self._log(f"TX SET_GAINS mode={self.var_g_mode.get()} seq={seq} kp={kp} ki={ki} kd={kd}")

    def send_mgmt(self, cmd: int) -> None:
        w = self._need_worker()
        if w is None:
            return
        seq = self._next_seq()
        w.send(w.ids["mgmt"], proto.pack_mgmt(cmd, seq))
        self._log(f"TX {proto.CMD_NAME.get(cmd, hex(cmd))} seq={seq}")

    def send_set_mode(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        mode_map = {"MOTION": proto.MODE_MOTION, "VELOCITY": proto.MODE_VELOCITY, "POSITION": proto.MODE_POSITION}
        seq = self._next_seq()
        mode = mode_map[self.var_set_mode.get()]
        w.send(w.ids["mgmt"], proto.pack_mgmt(proto.CMD_SET_MODE, seq, mode))
        self._log(f"TX SET_CONTROL_MODE {self.var_set_mode.get()} seq={seq}")

    def _poll_rx(self) -> None:
        try:
            self._refresh_snap()
        except ValueError:
            pass
        try:
            while True:
                kind, payload = self.rx_q.get_nowait()
                if kind == "fb":
                    self.var_fb_pos.set(f"{payload['pos']:.4f}")
                    self.var_fb_vel.set(f"{payload['vel']:.3f}")
                    self.var_fb_t.set(f"{payload['torque']:.3f}")
                    self.fb_count += 1
                    dt = time.monotonic() - self.fb_t0
                    if dt >= 0.5:
                        self.var_fb_hz.set(f"{self.fb_count / dt:.0f} Hz")
                        self.fb_count = 0
                        self.fb_t0 = time.monotonic()
                elif kind == "ack":
                    self.var_state.set(str(payload["state_name"]))
                    self.var_fault.set(str(payload["fault_name"]))
                    self._log(
                        f"ACK {payload['cmd_name']} seq={payload['seq']} "
                        f"{payload['result_name']} state={payload['state_name']} "
                        f"fault={payload['fault_name']}"
                    )
                elif kind == "status":
                    self.var_state.set(str(payload["state_name"]))
                    self.var_fault.set(str(payload["fault_name"]))
                    self.var_mode.set(str(payload["mode_name"]))
                    self._log(
                        f"STATUS {payload['state_name']} mode={payload['mode_name']} "
                        f"fault={payload['fault_name']} Vbus={payload['vbus']:.2f}V "
                        f"T={payload['temp']:.1f}C"
                    )
                elif kind == "cali":
                    self._log(
                        f"CALI seq={payload['seq']} {payload['state_name']} "
                        f"{payload['progress']}% stage={payload['stage_name']} err={payload['error']}"
                    )
                elif kind == "error":
                    self._log(f"ERROR {payload}")
        except queue.Empty:
            pass
        self.root.after(30, self._poll_rx)


def main() -> int:
    print("starting YFOCV3 GUI ...", flush=True)
    root = tk.Tk()
    root.minsize(900, 640)
    HostGui(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
