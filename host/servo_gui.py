#!/usr/bin/env python3
"""YFOCV3 host GUI: Motor CAN Protocol V1.0 + pyqtgraph scope."""

from __future__ import annotations

import queue
import sys
import threading
import time

try:
    import can
except ImportError:
    print("python-can is required: pip install python-can", file=sys.stderr)
    raise

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import can_backends
import protocol as proto
from can_worker import CanWorker
from scope_view import ScopeView

STYLE = """
QMainWindow, QWidget { background: #16181d; color: #e6e8ee; font-size: 13px; }
QLabel { color: #e6e8ee; }
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {
    background: #22262e; color: #e6e8ee; border: 1px solid #333844;
    border-radius: 6px; padding: 5px 8px; selection-background-color: #2d6cdf;
}
QComboBox QAbstractItemView { background: #22262e; color: #e6e8ee; }
QPushButton {
    background: #2d6cdf; color: white; border: none; border-radius: 6px;
    padding: 6px 14px; font-weight: 600;
}
QPushButton:hover { background: #3b7aee; }
QPushButton:pressed { background: #2458b8; }
QPushButton:checked { background: #c47b16; }
QPushButton#ghost, QPushButton#ghost:hover {
    background: #2a2f38; color: #d0d4dc; border: 1px solid #3a404c;
}
QTabWidget::pane { border: 1px solid #2c313c; border-radius: 8px; top: -1px; }
QTabBar::tab {
    background: #1c2028; color: #9aa3b2; padding: 8px 16px; margin-right: 2px;
    border-top-left-radius: 8px; border-top-right-radius: 8px;
}
QTabBar::tab:selected { background: #2a3140; color: #ffffff; }
QFrame#card {
    background: #1c2028; border: 1px solid #2c313c; border-radius: 10px;
}
QLabel#metric { color: #8b93a3; font-size: 11px; }
QLabel#metricVal { color: #f2f4f8; font-size: 20px; font-weight: 700; }
QLabel#connOk { color: #69f0ae; font-weight: 600; }
QLabel#connOff { color: #8b93a3; }
QLabel#scopeTitle { font-size: 15px; font-weight: 700; }
QLabel#scopeHint { color: #8b93a3; padding: 0 4px; }
QLabel#scopeHint[warn="true"] { color: #ffcc66; }
QLabel#scopeCursor { color: #c8cdd6; font-family: Consolas, "Cascadia Mono", monospace; font-size: 12px; }
QFrame#vsep { color: #333844; }
QCheckBox { color: #e6e8ee; spacing: 6px; }
QCheckBox::indicator { width: 14px; height: 14px; border-radius: 3px; border: 1px solid #4a5160; background: #22262e; }
QCheckBox::indicator:checked { background: #2d6cdf; border-color: #2d6cdf; }
QMenu {
    background: #1c2028; color: #e6e8ee; border: 1px solid #2c313c; padding: 4px;
}
QMenu::item { padding: 6px 18px; border-radius: 4px; }
QMenu::item:selected { background: #2d6cdf; }
QMenu::separator { height: 1px; background: #2c313c; margin: 4px 8px; }
QPlainTextEdit { font-family: Consolas, "Cascadia Mono", monospace; font-size: 12px; }
QSplitter::handle { background: #2c313c; height: 6px; }
"""


class HostWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("YFOCV3 Motor Host")
        self.resize(1280, 860)
        self.bus: can.BusABC | None = None
        self.worker: CanWorker | None = None
        self.rx_q: queue.Queue = queue.Queue()
        self.seq = 1
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self.fb_hz = 0.0
        self._snap = (0.0, 0.0, 0.0)
        self._vel_snap = 0.0
        self._pos_snap = (0.0, 100.0)
        self._snap_lock = threading.Lock()
        self._build()
        self._timer = QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._poll_rx)
        self._timer.start()

    def _build(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)
        layout.addWidget(self._conn_bar())
        layout.addWidget(self._metrics())

        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(self._tabs())
        scope_card = QFrame()
        scope_card.setObjectName("card")
        scope_lay = QVBoxLayout(scope_card)
        scope_lay.setContentsMargins(0, 0, 0, 0)
        self.scope = ScopeView()
        scope_lay.addWidget(self.scope)
        split.addWidget(scope_card)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 3)
        split.setSizes([280, 520])
        layout.addWidget(split, 1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(140)
        layout.addWidget(self.log)

    def _combo(self, items: list[str], width: int, editable: bool = True) -> QComboBox:
        box = QComboBox()
        box.setEditable(editable)
        box.addItems(items)
        box.setMinimumWidth(width)
        box.setMaximumWidth(width + 40)
        if editable and box.lineEdit() is not None:
            box.lineEdit().setPlaceholderText("可手填")
        return box

    def _conn_bar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("card")
        row = QHBoxLayout(bar)
        default_iface = can_backends.default_interface()
        self.cmb_iface = self._combo(list(can_backends.INTERFACES), 110)
        if self.cmb_iface.findText(default_iface) < 0:
            self.cmb_iface.insertItem(0, default_iface)
        self.cmb_iface.setCurrentText(default_iface)
        self.cmb_ch = self._combo(can_backends.list_channels(default_iface), 170)
        self.cmb_ch.setCurrentText(can_backends.default_channel(default_iface))
        self.cmb_bitrate = self._combo([str(b) for b in can_backends.BITRATES], 100)
        self.cmb_bitrate.setCurrentText("1000000")
        self.ed_id = QLineEdit("1")
        self.ed_id.setMaximumWidth(48)
        self.cmb_iface.activated.connect(self._on_iface_changed)
        row.addWidget(QLabel("接口"))
        row.addWidget(self.cmb_iface)
        row.addWidget(QLabel("通道"))
        row.addWidget(self.cmb_ch)
        btn_scan = QPushButton("扫描")
        btn_scan.setObjectName("ghost")
        btn_scan.clicked.connect(self._scan_channels)
        row.addWidget(btn_scan)
        row.addWidget(QLabel("波特率"))
        row.addWidget(self.cmb_bitrate)
        row.addWidget(QLabel("ID"))
        row.addWidget(self.ed_id)
        btn_on = QPushButton("连接")
        btn_off = QPushButton("断开")
        btn_off.setObjectName("ghost")
        btn_on.clicked.connect(self.connect_bus)
        btn_off.clicked.connect(self.disconnect_bus)
        row.addWidget(btn_on)
        row.addWidget(btn_off)
        self.lbl_conn = QLabel("未连接")
        self.lbl_conn.setObjectName("connOff")
        row.addWidget(self.lbl_conn)
        row.addStretch(1)
        return bar

    def _fill_channels(self, keep: str = "") -> None:
        iface = self.cmb_iface.currentText().strip()
        prev = keep or self.cmb_ch.currentText().strip()
        channels = can_backends.list_channels(iface)
        self.cmb_ch.blockSignals(True)
        self.cmb_ch.clear()
        self.cmb_ch.addItems(channels)
        if prev:
            idx = self.cmb_ch.findText(prev)
            if idx >= 0:
                self.cmb_ch.setCurrentIndex(idx)
            else:
                self.cmb_ch.setEditText(prev)
        elif channels:
            self.cmb_ch.setCurrentIndex(0)
        self.cmb_ch.blockSignals(False)

    def _on_iface_changed(self, _text: str = "") -> None:
        iface = self.cmb_iface.currentText().strip()
        self._fill_channels(can_backends.default_channel(iface))

    def _scan_channels(self) -> None:
        iface = self.cmb_iface.currentText().strip()
        found = can_backends.detect_channels(iface)
        self._fill_channels(self.cmb_ch.currentText().strip())
        if found:
            self._log(f"扫描 {iface}: " + ", ".join(found))
        else:
            self._log(f"扫描 {iface}: 未检测到设备，已列出常用通道，可手填")

    def _metrics(self) -> QFrame:
        box = QFrame()
        box.setObjectName("card")
        grid = QGridLayout(box)
        self.val_pos = QLabel("—")
        self.val_vel = QLabel("—")
        self.val_volt = QLabel("—")
        self.val_hz = QLabel("0 Hz")
        self.val_state = QLabel("—")
        self.val_fault = QLabel("—")
        self.val_mode = QLabel("—")
        items = (
            ("位置 rad", self.val_pos),
            ("速度 rad/s", self.val_vel),
            ("电压 pu", self.val_volt),
            ("反馈", self.val_hz),
            ("State", self.val_state),
            ("Fault", self.val_fault),
            ("Mode", self.val_mode),
        )
        for col, (name, val) in enumerate(items):
            lab = QLabel(name)
            lab.setObjectName("metric")
            val.setObjectName("metricVal")
            grid.addWidget(lab, 0, col)
            grid.addWidget(val, 1, col)
        return box

    def _tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        tabs.addTab(self._tab_motion(), "Motion")
        tabs.addTab(self._tab_vel(), "Velocity")
        tabs.addTab(self._tab_pos(), "Position")
        tabs.addTab(self._tab_gains(), "Gains")
        tabs.addTab(self._tab_mgmt(), "Management")
        return tabs

    def _form(self, parent: QWidget, rows: list[tuple[str, QWidget]]) -> QFormLayout:
        form = QFormLayout(parent)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        for label, widget in rows:
            form.addRow(label, widget)
        return form

    def _tab_motion(self) -> QWidget:
        tab = QWidget()
        self.ed_m_pos = QLineEdit("0.0")
        self.ed_m_vel = QLineEdit("0.0")
        self.ed_m_ff = QLineEdit("0.0")
        self.ed_m_rate = QLineEdit("200")
        self._form(
            tab,
            [
                ("位置 rad", self.ed_m_pos),
                ("速度 rad/s", self.ed_m_vel),
                ("Voltage FF 0~1", self.ed_m_ff),
                ("循环频率 Hz", self.ed_m_rate),
            ],
        )
        btns = QHBoxLayout()
        for text, slot in (
            ("发送一次", self.send_motion_once),
            ("开始循环", self.start_cyclic_motion),
            ("停止循环", self.stop_cyclic),
            ("用反馈位置填入", self.fill_pos_from_fb),
        ):
            b = QPushButton(text)
            if text == "停止循环":
                b.setObjectName("ghost")
            b.clicked.connect(slot)
            btns.addWidget(b)
        btns.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(btns)
        tab.layout().addRow(wrap)
        hint = QLabel("循环发送才会持续触发 0x300 反馈，示波才能跟上。")
        hint.setObjectName("scopeHint")
        tab.layout().addRow(hint)
        return tab

    def _tab_vel(self) -> QWidget:
        tab = QWidget()
        self.ed_v_vel = QLineEdit("0.0")
        self.ed_v_rate = QLineEdit("200")
        self._form(tab, [("目标速度 rad/s", self.ed_v_vel), ("循环频率 Hz", self.ed_v_rate)])
        btns = QHBoxLayout()
        for text, slot in (
            ("发送一次", self.send_velocity),
            ("开始循环", self.start_cyclic_vel),
            ("停止循环", self.stop_cyclic),
        ):
            b = QPushButton(text)
            if text == "停止循环":
                b.setObjectName("ghost")
            b.clicked.connect(slot)
            btns.addWidget(b)
        btns.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(btns)
        tab.layout().addRow(wrap)
        hint = QLabel("先 SET_CONTROL_MODE=VELOCITY，并设置 VELOCITY Gains。")
        hint.setObjectName("scopeHint")
        tab.layout().addRow(hint)
        return tab

    def _tab_pos(self) -> QWidget:
        tab = QWidget()
        self.ed_p_pos = QLineEdit("0.0")
        self.ed_p_maxv = QLineEdit("100.0")
        self.ed_p_rate = QLineEdit("200")
        self._form(
            tab,
            [
                ("目标位置 rad", self.ed_p_pos),
                ("最大速度 rad/s", self.ed_p_maxv),
                ("循环频率 Hz", self.ed_p_rate),
            ],
        )
        btns = QHBoxLayout()
        for text, slot in (
            ("发送一次", self.send_position),
            ("开始循环", self.start_cyclic_pos),
            ("停止循环", self.stop_cyclic),
            ("用反馈位置填入", self.fill_pos_cmd_from_fb),
        ):
            b = QPushButton(text)
            if text == "停止循环":
                b.setObjectName("ghost")
            b.clicked.connect(slot)
            btns.addWidget(b)
        btns.addStretch(1)
        wrap = QWidget()
        wrap.setLayout(btns)
        tab.layout().addRow(wrap)
        hint = QLabel("上电默认位置模式。先配 VELOCITY / POSITION Gains。")
        hint.setObjectName("scopeHint")
        tab.layout().addRow(hint)
        return tab

    def _tab_gains(self) -> QWidget:
        tab = QWidget()
        self.cmb_g_mode = QComboBox()
        self.cmb_g_mode.addItems(("MOTION", "VELOCITY", "POSITION"))
        self.ed_g_kp = QLineEdit("0.0")
        self.ed_g_ki = QLineEdit("0.0")
        self.ed_g_kd = QLineEdit("0.0")
        self._form(
            tab,
            [
                ("参数组", self.cmb_g_mode),
                ("Kp", self.ed_g_kp),
                ("Ki", self.ed_g_ki),
                ("Kd", self.ed_g_kd),
            ],
        )
        btn = QPushButton("发送 SET_GAINS")
        btn.clicked.connect(self.send_gains)
        wrap = QWidget()
        row = QHBoxLayout(wrap)
        row.setContentsMargins(0, 0, 0, 0)
        row.addWidget(btn)
        row.addStretch(1)
        tab.layout().addRow(wrap)
        hint = QLabel("MOTION：Ki=0。VELOCITY：Kd=0。POSITION 只改外环，内环用 VELOCITY PI。")
        hint.setObjectName("scopeHint")
        tab.layout().addRow(hint)
        return tab

    def _tab_mgmt(self) -> QWidget:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        row = QHBoxLayout()
        for text, cmd in (
            ("ENABLE", proto.CMD_ENABLE),
            ("DISABLE", proto.CMD_DISABLE),
            ("SET_ZERO", proto.CMD_SET_ZERO),
            ("CLEAR_FAULT", proto.CMD_CLEAR_FAULT),
            ("START_CALI", proto.CMD_START_CALI),
            ("GET_STATUS", proto.CMD_GET_STATUS),
        ):
            b = QPushButton(text)
            if text in ("ENABLE", "DISABLE"):
                b.setObjectName("ghost")
            b.clicked.connect(lambda _=False, c=cmd: self.send_mgmt(c))
            row.addWidget(b)
        row.addStretch(1)
        lay.addLayout(row)
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("SET_CONTROL_MODE"))
        self.cmb_set_mode = QComboBox()
        self.cmb_set_mode.addItems(("MOTION", "VELOCITY", "POSITION"))
        self.cmb_set_mode.setCurrentText("POSITION")
        mode_row.addWidget(self.cmb_set_mode)
        btn = QPushButton("发送")
        btn.clicked.connect(self.send_set_mode)
        mode_row.addWidget(btn)
        mode_row.addStretch(1)
        lay.addLayout(mode_row)
        hint = QLabel(
            "上电默认 POSITION。ENABLE/DISABLE 仍返回 INVALID_COMMAND。"
            "校准前先停止循环发送。"
        )
        hint.setObjectName("scopeHint")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        lay.addStretch(1)
        return tab

    def _log(self, text: str) -> None:
        self.log.appendPlainText(text)

    def _need_worker(self) -> CanWorker | None:
        if self.worker is None:
            QMessageBox.warning(self, "未连接", "请先连接 CAN")
            return None
        return self.worker

    def _next_seq(self) -> int:
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def _f(self, edit: QLineEdit) -> float:
        return float(edit.text().strip())

    def connect_bus(self) -> None:
        self.disconnect_bus()
        iface = self.cmb_iface.currentText().strip()
        channel = self.cmb_ch.currentText().strip()
        try:
            node_id = int(self.ed_id.text())
            bitrate = int(self.cmb_bitrate.currentText().strip())
            bus = can_backends.open_bus(iface, channel, bitrate)
        except Exception as exc:
            QMessageBox.critical(self, "连接失败", str(exc))
            return
        self.bus = bus
        self.rx_q = queue.Queue()
        self.worker = CanWorker(bus, node_id, self.rx_q)
        self.worker.start()
        self.lbl_conn.setText(f"已连接  {iface}:{channel}  ID={node_id}")
        self.lbl_conn.setObjectName("connOk")
        self.lbl_conn.style().unpolish(self.lbl_conn)
        self.lbl_conn.style().polish(self.lbl_conn)
        self._log(f"connected iface={iface} ch={channel} bitrate={self.cmb_bitrate.currentText()} id={node_id}")

    def disconnect_bus(self) -> None:
        if self.worker is not None:
            self.worker.set_cyclic(False, 1.0, 0, lambda: proto.pack_motion(0.0, 0.0, 0.0))
            self.worker.stop()
            self.worker.join(timeout=0.5)
            self.worker = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.lbl_conn.setText("未连接")
        self.lbl_conn.setObjectName("connOff")
        self.lbl_conn.style().unpolish(self.lbl_conn)
        self.lbl_conn.style().polish(self.lbl_conn)
        self.scope.set_status(False, self.fb_hz)

    def closeEvent(self, event) -> None:
        self.disconnect_bus()
        super().closeEvent(event)

    def _refresh_snap(self) -> bytes:
        pos = self._f(self.ed_m_pos)
        vel = self._f(self.ed_m_vel)
        ff = self._f(self.ed_m_ff)
        with self._snap_lock:
            self._snap = (pos, vel, ff)
        return proto.pack_motion(pos, vel, ff)

    def _refresh_vel_pos_snap(self) -> None:
        try:
            vel = self._f(self.ed_v_vel)
            ppos = self._f(self.ed_p_pos)
            pmax = self._f(self.ed_p_maxv)
        except ValueError:
            return
        with self._snap_lock:
            self._vel_snap = vel
            self._pos_snap = (ppos, pmax)

    def _motion_payload(self) -> bytes:
        with self._snap_lock:
            pos, vel, ff = self._snap
        return proto.pack_motion(pos, vel, ff)

    def _vel_payload(self) -> bytes:
        with self._snap_lock:
            vel = self._vel_snap
        return proto.pack_velocity(vel)

    def _pos_payload(self) -> bytes:
        with self._snap_lock:
            pos, maxv = self._pos_snap
        return proto.pack_position(pos, maxv)

    def send_motion_once(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = self._refresh_snap()
        except ValueError:
            QMessageBox.critical(self, "参数错误", "位置/速度/FF 必须是数字")
            return
        w.send(w.ids["motion"], data)
        self._log(f"TX Motion pos={self.ed_m_pos.text()} vel={self.ed_m_vel.text()} ff={self.ed_m_ff.text()}")

    def start_cyclic_motion(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            rate = float(self.ed_m_rate.text())
            self._refresh_snap()
        except ValueError:
            QMessageBox.critical(self, "参数错误", "频率和 Motion 参数必须是数字")
            return
        w.set_cyclic(True, rate, w.ids["motion"], self._motion_payload)
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self._log(f"cyclic Motion {rate:.1f} Hz")

    def start_cyclic_vel(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            rate = float(self.ed_v_rate.text())
            vel = self._f(self.ed_v_vel)
        except ValueError:
            QMessageBox.critical(self, "参数错误", "频率和速度必须是数字")
            return
        self._refresh_vel_pos_snap()
        w.set_cyclic(True, rate, w.ids["vel"], self._vel_payload)
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self._log(f"cyclic Velocity {rate:.1f} Hz vel={vel}")

    def start_cyclic_pos(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            rate = float(self.ed_p_rate.text())
            pos = self._f(self.ed_p_pos)
            maxv = self._f(self.ed_p_maxv)
        except ValueError:
            QMessageBox.critical(self, "参数错误", "频率和位置参数必须是数字")
            return
        self._refresh_vel_pos_snap()
        w.set_cyclic(True, rate, w.ids["pos"], self._pos_payload)
        self.fb_count = 0
        self.fb_t0 = time.monotonic()
        self._log(f"cyclic Position {rate:.1f} Hz pos={pos} max_vel={maxv}")

    def stop_cyclic(self) -> None:
        if self.worker is not None:
            self.worker.set_cyclic(False, 1.0, 0, self._motion_payload)
        self._log("cyclic stopped")

    def fill_pos_from_fb(self) -> None:
        if self.val_pos.text() not in ("—", ""):
            self.ed_m_pos.setText(self.val_pos.text())

    def fill_pos_cmd_from_fb(self) -> None:
        if self.val_pos.text() not in ("—", ""):
            self.ed_p_pos.setText(self.val_pos.text())

    def send_velocity(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = proto.pack_velocity(self._f(self.ed_v_vel))
        except ValueError:
            QMessageBox.critical(self, "参数错误", "速度必须是数字")
            return
        w.send(w.ids["vel"], data)
        self._log(f"TX Velocity vel={self.ed_v_vel.text()}")

    def send_position(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        try:
            data = proto.pack_position(self._f(self.ed_p_pos), self._f(self.ed_p_maxv))
        except ValueError:
            QMessageBox.critical(self, "参数错误", "位置/最大速度必须是数字")
            return
        w.send(w.ids["pos"], data)
        self._log(f"TX Position pos={self.ed_p_pos.text()} max_vel={self.ed_p_maxv.text()}")

    def send_gains(self) -> None:
        w = self._need_worker()
        if w is None:
            return
        mode_map = {"MOTION": proto.MODE_MOTION, "VELOCITY": proto.MODE_VELOCITY, "POSITION": proto.MODE_POSITION}
        try:
            kp, ki, kd = self._f(self.ed_g_kp), self._f(self.ed_g_ki), self._f(self.ed_g_kd)
        except ValueError:
            QMessageBox.critical(self, "参数错误", "Kp/Ki/Kd 必须是数字")
            return
        seq = self._next_seq()
        mode = mode_map[self.cmb_g_mode.currentText()]
        w.send(w.ids["gains"], proto.pack_gains(mode, seq, kp, ki, kd))
        self._log(f"TX SET_GAINS mode={self.cmb_g_mode.currentText()} seq={seq} kp={kp} ki={ki} kd={kd}")

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
        mode = mode_map[self.cmb_set_mode.currentText()]
        w.send(w.ids["mgmt"], proto.pack_mgmt(proto.CMD_SET_MODE, seq, mode))
        self._log(f"TX SET_CONTROL_MODE {self.cmb_set_mode.currentText()} seq={seq}")

    def _poll_rx(self) -> None:
        try:
            self._refresh_snap()
        except ValueError:
            pass
        self._refresh_vel_pos_snap()
        cyclic = self.worker.is_cyclic() if self.worker is not None else False
        try:
            while True:
                kind, payload = self.rx_q.get_nowait()
                if kind == "fb":
                    self.val_pos.setText(f"{payload['pos']:.4f}")
                    self.val_vel.setText(f"{payload['vel']:.3f}")
                    self.val_volt.setText(f"{payload['torque']:.3f}")
                    self.scope.push_feedback(payload["pos"], payload["vel"], payload["torque"])
                    self.fb_count += 1
                    dt = time.monotonic() - self.fb_t0
                    if dt >= 0.5:
                        self.fb_hz = self.fb_count / dt
                        self.val_hz.setText(f"{self.fb_hz:.0f} Hz")
                        self.fb_count = 0
                        self.fb_t0 = time.monotonic()
                elif kind == "ack":
                    self.val_state.setText(str(payload["state_name"]))
                    self.val_fault.setText(str(payload["fault_name"]))
                    self._log(
                        f"ACK {payload['cmd_name']} seq={payload['seq']} "
                        f"{payload['result_name']} state={payload['state_name']} "
                        f"fault={payload['fault_name']}"
                    )
                elif kind == "status":
                    self.val_state.setText(str(payload["state_name"]))
                    self.val_fault.setText(str(payload["fault_name"]))
                    self.val_mode.setText(str(payload["mode_name"]))
                    self._log(
                        f"STATUS {payload['state_name']} mode={payload['mode_name']} "
                        f"fault={payload['fault_name']} Vbus={payload['vbus']:.2f}V "
                        f"T={payload['temp']:.1f}C"
                    )
                elif kind == "cali":
                    self._log(
                        f"CALI seq={payload['seq']} {payload['state_name']} "
                        f"{payload['progress']}% stage={payload['stage_name']} "
                        f"err={payload['error_name']}"
                    )
                elif kind == "error":
                    self._log(f"ERROR {payload}")
        except queue.Empty:
            pass
        self.scope.set_status(cyclic, self.fb_hz)


def main() -> int:
    print("starting YFOCV3 GUI ...", flush=True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Segoe UI", 10))
    app.setStyleSheet(STYLE)
    win = HostWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
