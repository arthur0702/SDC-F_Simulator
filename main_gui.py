"""
士林電機 SDC-F 多軸伺服驅動器模擬器 — 圖形化監控與控制主程式 (PySide6 + qasync)
================================================================================

提供工業深色介面與即時監控功能：
- 多軸動態卡片網格：馬達轉子旋轉動畫、線性滑台、CSR/SSR LED、PDS 狀態、即時位置/速度/追隨誤差
- 虛擬硬體極限與感測器控制：LSN / ORGP / LSP 與異警注入/復歸
- 右側分頁式診斷面板：
  1. PDO 監控 (RWwn / RWrn 32-Word 即時解碼)
  2. SLMP 物件字典瀏覽與雙擊修改 (1000h ~ 6FFFh)
  3. SLMP 與伺服狀態事件通訊日誌
  4. 內建 SLMP 4020h (UDP Port 5010) 封包測試發送器
- Windows 多 IP 綁定與防火牆一鍵 netsh 批次腳本 (.bat) 產生器
"""

from __future__ import annotations

import asyncio
import ipaddress
import struct
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional

import qasync
from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from iefb_cyclic_server import (
    AxisNetworkBundle,
    CyclicFrameConfig,
    MultiAxisCyclicManager,
    diagnose_local_network,
    generate_windows_netsh_script,
    generate_windows_restore_script,
)
from object_dictionary import AccessType, DataType, ENCODER_PPR, SLMP_ERROR_MESSAGES, SLMPError
from servo_axis import OpMode, PDSState, ServoAxis
from slmp_server import (
    SLMP_SUB_READ_BLOCK,
    SLMP_SUB_READ_OBJ,
    SLMP_SUB_WRITE_BLOCK,
    SLMP_SUB_WRITE_OBJ,
    SLMPClientTester,
    SLMPTransaction,
)


DARK_INDUSTRIAL_QSS = """
QMainWindow, QWidget {
    background-color: #12151B;
    color: #E2E8F0;
    font-family: "Segoe UI", "Microsoft JhengHei UI", "Consolas", sans-serif;
    font-size: 12px;
}
QGroupBox {
    background-color: #181C24;
    border: 1px solid #2D3545;
    border-radius: 6px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
    color: #94A3B8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #38BDF8;
}
QPushButton {
    background-color: #232A38;
    border: 1px solid #384359;
    border-radius: 4px;
    padding: 5px 10px;
    color: #E2E8F0;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #2E3749;
    border-color: #38BDF8;
}
QPushButton:pressed {
    background-color: #1B212C;
}
QPushButton:checked {
    background-color: #0284C7;
    border-color: #38BDF8;
    color: #FFFFFF;
}
QPushButton#primaryBtn {
    background-color: #0369A1;
    border-color: #0EA5E9;
    color: #FFFFFF;
}
QPushButton#primaryBtn:hover {
    background-color: #0284C7;
}
QPushButton#dangerBtn {
    background-color: #7F1D1D;
    border-color: #EF4444;
    color: #FEE2E2;
}
QPushButton#dangerBtn:hover {
    background-color: #991B1B;
}
QPushButton#successBtn {
    background-color: #065F46;
    border-color: #10B981;
    color: #ECFDF5;
}
QPushButton#successBtn:hover {
    background-color: #047857;
}
QLineEdit, QComboBox, QSpinBox {
    background-color: #0F1218;
    border: 1px solid #334155;
    border-radius: 4px;
    padding: 4px 7px;
    color: #F1F5F9;
    selection-background-color: #0284C7;
}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus {
    border-color: #38BDF8;
}
QTableWidget {
    background-color: #0F1218;
    alternate-background-color: #151922;
    gridline-color: #242B38;
    border: 1px solid #2D3545;
    color: #E2E8F0;
    font-family: "Consolas", "Microsoft JhengHei UI", monospace;
    font-size: 11px;
}
QHeaderView::section {
    background-color: #1E2430;
    color: #94A3B8;
    border: 1px solid #2D3545;
    padding: 4px 6px;
    font-weight: bold;
}
QTabWidget::pane {
    border: 1px solid #2D3545;
    background-color: #181C24;
    border-radius: 4px;
}
QTabBar::tab {
    background-color: #161A22;
    border: 1px solid #2D3545;
    border-bottom: none;
    padding: 6px 14px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    color: #94A3B8;
}
QTabBar::tab:selected {
    background-color: #1E2532;
    color: #38BDF8;
    border-color: #38BDF8;
    font-weight: bold;
}
QPlainTextEdit {
    background-color: #0B0E14;
    border: 1px solid #2D3545;
    color: #CBD5E1;
    font-family: "Consolas", monospace;
    font-size: 11px;
}
QScrollArea {
    border: none;
    background-color: #12151B;
}
QSplitter::handle {
    background-color: #2D3545;
    border-radius: 2px;
    margin: 2px 1px;
}
QSplitter::handle:hover {
    background-color: #38BDF8;
}
"""


class LEDIndicator(QWidget):
    """圓點 LED 指示燈元件（用於顯示 CSR / SSR 即時狀態）。"""

    def __init__(self, label: str, active_color: str = "#10B981", parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._active = False
        self._active_color = QColor(active_color)
        self._inactive_color = QColor("#334155")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 0, 4, 0)
        layout.setSpacing(5)
        self._dot = _LEDDot(self._active_color, self._inactive_color)
        self._lbl = QLabel(label)
        self._lbl.setStyleSheet("color: #CBD5E1; font-size: 11px; font-weight: 600;")
        layout.addWidget(self._dot)
        layout.addWidget(self._lbl)

    def set_active(self, active: bool) -> None:
        if self._active != active:
            self._active = active
            self._dot.set_active(active)


class _LEDDot(QWidget):
    """自繪圓形發光燈號小元件。"""

    def __init__(self, active_color: QColor, inactive_color: QColor) -> None:
        super().__init__()
        self.setFixedSize(12, 12)
        self._active = False
        self._active_color = active_color
        self._inactive_color = inactive_color

    def set_active(self, active: bool) -> None:
        self._active = active
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = self._active_color if self._active else self._inactive_color
        p.setPen(QPen(c.lighter(130) if self._active else QColor("#475569"), 1.2))
        p.setBrush(QBrush(c))
        p.drawEllipse(1, 1, 10, 10)


class MotorRotorWidget(QWidget):
    """伺服馬達轉子角度視覺化儀表（依 PDS 狀態切換外環顏色，指針即時顯示旋轉角度）。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFixedSize(84, 84)
        self._angle_deg = 0.0
        self._pds_state = PDSState.SWITCH_ON_DISABLED

    def set_state(self, angle_deg: float, pds_state: PDSState) -> None:
        self._angle_deg = angle_deg
        self._pds_state = pds_state
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        import math
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        cx, cy = rect.width() / 2.0, rect.height() / 2.0
        radius = min(cx, cy) - 6.0

        if self._pds_state == PDSState.OPERATION_ENABLED:
            ring_color = QColor("#10B981")
        elif self._pds_state in (PDSState.READY_TO_SWITCH_ON, PDSState.SWITCHED_ON):
            ring_color = QColor("#F59E0B")
        elif self._pds_state == PDSState.FAULT:
            ring_color = QColor("#EF4444")
        else:
            ring_color = QColor("#475569")

        p.setPen(QPen(ring_color, 3.0))
        p.setBrush(QBrush(QColor("#0F131A")))
        p.drawEllipse(QPointF(cx, cy), radius, radius)

        inner_r = radius * 0.72
        p.setPen(QPen(QColor("#334155"), 1.5))
        p.setBrush(QBrush(QColor("#1E293B")))
        p.drawEllipse(QPointF(cx, cy), inner_r, inner_r)

        p.setPen(QPen(QColor("#475569"), 1.2))
        for deg in (0, 90, 180, 270):
            tick_rad = math.radians(deg)
            p.drawLine(
                QPointF(cx + math.cos(tick_rad) * (radius - 5), cy + math.sin(tick_rad) * (radius - 5)),
                QPointF(cx + math.cos(tick_rad) * (radius - 1), cy + math.sin(tick_rad) * (radius - 1)),
            )

        rad = math.radians(self._angle_deg - 90.0)
        tip_x = cx + math.cos(rad) * (inner_r - 2)
        tip_y = cy + math.sin(rad) * (inner_r - 2)
        p.setPen(QPen(QColor("#38BDF8"), 3.0))
        p.drawLine(QPointF(cx, cy), QPointF(tip_x, tip_y))

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(ring_color))
        p.drawEllipse(QPointF(cx, cy), 4.5, 4.5)


class LinearSlideWidget(QWidget):
    """線性滑台與負極限 (LSN)、原點犬 (ORGP)、正極限 (LSP) 位置視覺化指示條。"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setFixedHeight(28)
        self._pos = 0
        self._min_range = -1000000
        self._max_range = 1000000
        self._lsn = False
        self._lsp = False
        self._orgp = False

    def set_position(self, pos: int, lsn: bool, orgp: bool, lsp: bool) -> None:
        self._pos = pos
        self._lsn = lsn
        self._orgp = orgp
        self._lsp = lsp
        span = max(1000000, abs(pos) + 100000)
        self._min_range = -span
        self._max_range = span
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width()
        h = self.height()

        track_rect = QRectF(8, h / 2.0 - 4, w - 16, 8)
        p.setPen(QPen(QColor("#334155"), 1.0))
        p.setBrush(QBrush(QColor("#0B0E14")))
        p.drawRoundedRect(track_rect, 3, 3)

        mid_x = w / 2.0
        p.setPen(QPen(QColor("#F59E0B") if self._orgp else QColor("#475569"), 2.0))
        p.drawLine(QPointF(mid_x, 3), QPointF(mid_x, h - 3))

        p.setBrush(QBrush(QColor("#EF4444") if self._lsn else QColor("#334155")))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawRect(2, 6, 5, h - 12)
        p.setBrush(QBrush(QColor("#EF4444") if self._lsp else QColor("#334155")))
        p.drawRect(w - 7, 6, 5, h - 12)

        ratio = (self._pos - self._min_range) / max(1, (self._max_range - self._min_range))
        ratio = max(0.0, min(1.0, ratio))
        cx = 10.0 + ratio * (w - 20.0)
        carriage = QRectF(cx - 7, 4, 14, h - 8)
        p.setPen(QPen(QColor("#7DD3FC"), 1.2))
        p.setBrush(QBrush(QColor("#0284C7")))
        p.drawRoundedRect(carriage, 3, 3)


class AxisCardWidget(QFrame):
    """單軸伺服控制與監控卡片：顯示軸 IP、通訊狀態、馬達動態、感測器按鈕及異警注入控制。"""

    selected = Signal(int)
    ip_change_requested = Signal(int, str)

    def __init__(self, bundle: AxisNetworkBundle, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.bundle = bundle
        self.axis = bundle.axis
        self._is_selected = False
        self._last_synced_ip = self.axis.ip_address
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.setMinimumWidth(410)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._apply_border()

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 8)
        root.setSpacing(6)

        hdr = QHBoxLayout()
        hdr.setSpacing(4)
        self.lbl_title = QLabel(f"Axis {self.axis.axis_id}")
        self.lbl_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #38BDF8;")

        self.edt_ip = QLineEdit(self.axis.ip_address)
        self.edt_ip.setFixedWidth(104)
        self.edt_ip.setToolTip("輸入此軸 IP 位址並按 Enter 或點擊「改IP」套用")
        self.edt_ip.returnPressed.connect(self._emit_ip_change)

        self.btn_set_ip = QPushButton("改IP")
        self.btn_set_ip.setFixedHeight(22)
        self.btn_set_ip.setToolTip("套用此軸新 IP 位址並重啟該軸 UDP 服務")
        self.btn_set_ip.clicked.connect(self._emit_ip_change)

        self.lbl_net = QLabel("UDP: OFF")
        self.lbl_net.setStyleSheet("font-size: 10px; color: #94A3B8; font-family: Consolas;")
        self.led_csr = LEDIndicator("CSR(RY)", "#38BDF8")
        self.led_ssr = LEDIndicator("SSR(RX)", "#10B981")
        hdr.addWidget(self.lbl_title)
        hdr.addWidget(self.edt_ip)
        hdr.addWidget(self.btn_set_ip)
        hdr.addStretch()
        hdr.addWidget(self.led_csr)
        hdr.addWidget(self.led_ssr)
        hdr.addWidget(self.lbl_net)
        root.addLayout(hdr)

        mid = QHBoxLayout()
        self.rotor = MotorRotorWidget()
        mid.addWidget(self.rotor)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(2)

        def _mk_val(color: str = "#F8FAFC") -> QLabel:
            lbl = QLabel("0")
            lbl.setStyleSheet(f"font-family: Consolas; font-size: 12px; font-weight: bold; color: {color};")
            return lbl

        self.lbl_pds = _mk_val("#FBBF24")
        self.lbl_mode = _mk_val("#A5F3FC")
        self.lbl_pos = _mk_val("#34D399")
        self.lbl_ferr = _mk_val("#F472B6")
        self.lbl_vel = _mk_val("#60A5FA")
        self.lbl_vlim = _mk_val("#94A3B8")
        self.lbl_cw_sw = _mk_val("#CBD5E1")
        self.lbl_err = _mk_val("#F87171")

        labels = [
            ("PDS State:", self.lbl_pds, "6061h Mode:", self.lbl_mode),
            ("6064h Pos:", self.lbl_pos, "60F4h F.Err:", self.lbl_ferr),
            ("606Ch Vel:", self.lbl_vel, "2D20h VLim:", self.lbl_vlim),
            ("CW / SW:", self.lbl_cw_sw, "603Fh Err:", self.lbl_err),
        ]
        for r, (k1, v1, k2, v2) in enumerate(labels):
            lk1 = QLabel(k1)
            lk1.setStyleSheet("color: #94A3B8; font-size: 11px;")
            lk2 = QLabel(k2)
            lk2.setStyleSheet("color: #94A3B8; font-size: 11px;")
            grid.addWidget(lk1, r, 0)
            grid.addWidget(v1, r, 1)
            grid.addWidget(lk2, r, 2)
            grid.addWidget(v2, r, 3)

        mid.addLayout(grid, 1)
        root.addLayout(mid)

        self.slider = LinearSlideWidget()
        root.addWidget(self.slider)

        ctrl_sensors = QHBoxLayout()
        ctrl_sensors.setSpacing(4)

        self.btn_lsn = QPushButton("LSN")
        self.btn_lsn.setCheckable(True)
        self.btn_lsn.toggled.connect(lambda c: self.axis.set_sensor("lsn", c))

        self.btn_orgp = QPushButton("ORGP")
        self.btn_orgp.setCheckable(True)
        self.btn_orgp.toggled.connect(lambda c: self.axis.set_sensor("orgp", c))

        self.btn_lsp = QPushButton("LSP")
        self.btn_lsp.setCheckable(True)
        self.btn_lsp.toggled.connect(lambda c: self.axis.set_sensor("lsp", c))

        for w in (
            self.btn_lsn,
            self.btn_orgp,
            self.btn_lsp,
        ):
            w.setMaximumHeight(24)
            w.setMinimumWidth(40)
            ctrl_sensors.addWidget(w)
        ctrl_sensors.addStretch()
        root.addLayout(ctrl_sensors)

        ctrl_fault = QHBoxLayout()
        ctrl_fault.setSpacing(6)
        lbl_flt = QLabel("異警注入:")
        lbl_flt.setStyleSheet("color: #94A3B8; font-size: 11px;")
        ctrl_fault.addWidget(lbl_flt)

        self.cmb_fault = QComboBox()
        self.cmb_fault.addItem("AL.03 過電流", 0xFF03)
        self.cmb_fault.addItem("AL.08 誤差過大", 0xFF08)
        self.cmb_fault.addItem("AL.01 過電壓", 0xFF01)
        self.cmb_fault.addItem("AL.02 低電壓", 0xFF02)
        self.cmb_fault.addItem("AL.05 過負載", 0xFF05)
        self.cmb_fault.addItem("AL.06 過速度", 0xFF06)
        self.cmb_fault.setFixedWidth(125)
        self.cmb_fault.setMaximumHeight(24)

        self.btn_fault = QPushButton("注入異警")
        self.btn_fault.setObjectName("dangerBtn")
        self.btn_fault.setMaximumHeight(24)
        self.btn_fault.clicked.connect(self._inject_selected_fault)

        self.btn_clear_fault = QPushButton("異警復歸")
        self.btn_clear_fault.setMaximumHeight(24)
        self.btn_clear_fault.clicked.connect(self.axis.clear_fault)

        ctrl_fault.addWidget(self.cmb_fault)
        ctrl_fault.addWidget(self.btn_fault)
        ctrl_fault.addWidget(self.btn_clear_fault)
        ctrl_fault.addStretch()
        root.addLayout(ctrl_fault)

    def _inject_selected_fault(self) -> None:
        code = int(self.cmb_fault.currentData())
        name = self.cmb_fault.currentText()
        self.axis.inject_fault(code, name)

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self.selected.emit(self.axis.axis_id)
        super().mousePressEvent(event)

    def set_selected(self, selected: bool) -> None:
        self._is_selected = selected
        self._apply_border()

    def _apply_border(self) -> None:
        if self._is_selected:
            self.setStyleSheet(
                "AxisCardWidget { background-color: #1B2230; border: 2px solid #38BDF8; border-radius: 6px; }"
            )
        else:
            self.setStyleSheet(
                "AxisCardWidget { background-color: #161B24; border: 1px solid #2D3545; border-radius: 6px; }"
            )

    def _emit_ip_change(self) -> None:
        self.ip_change_requested.emit(self.axis.axis_id, self.edt_ip.text().strip())

    def refresh(self) -> None:
        snap = self.axis.get_snapshot()
        self.lbl_title.setText(f"Axis {self.axis.axis_id}")
        if self.axis.ip_address != self._last_synced_ip:
            self._last_synced_ip = self.axis.ip_address
            if not self.edt_ip.hasFocus():
                self.edt_ip.setText(self.axis.ip_address)

        net_ok = self.bundle.all_running
        cyclic_srv = self.bundle.cyclic_server
        stats = cyclic_srv.stats
        bound_ip = cyclic_srv.bound_ip or "-"
        if stats.rx_packets > 1 and stats.avg_scan_ms > 0.0:
            scan_info = f"{stats.avg_scan_ms:.1f}ms RX:{stats.rx_packets}"
        else:
            scan_info = f"RX:{snap['rx_count']}"
        if stats.malformed_packets > 0:
            scan_info += f" ERR:{stats.malformed_packets}"

        if not cyclic_srv.is_running and not self.bundle.slmp_server.is_running:
            net_text = "UDP: OFF (已停止)"
            net_color = "#64748B"
        elif not net_ok:
            net_text = f"UDP: PARTIAL ({scan_info})"
            net_color = "#F59E0B"
        elif stats.link_lost or (stats.last_rx_ts > 0.0 and not stats.is_link_alive(link_timeout_s=1.5)):
            idle_s = stats.idle_seconds()
            net_text = f"UDP: TIMEOUT! ({idle_s:.1f}s)"
            net_color = "#EF4444"
        elif stats.rx_packets == 0:
            net_text = f"UDP: WAIT ({bound_ip})"
            net_color = "#F59E0B"
        elif bound_ip == "127.0.0.1":
            net_text = f"UDP: LOOPBACK! ({scan_info})"
            net_color = "#EF4444"
        elif bound_ip == "0.0.0.0":
            net_text = f"UDP: ANY ({scan_info})"
            net_color = "#38BDF8"
        else:
            net_text = f"UDP: ON ({scan_info})"
            net_color = "#10B981"

        self.lbl_net.setText(net_text)
        self.lbl_net.setStyleSheet(
            f"font-size: 10px; font-family: Consolas; font-weight: bold; color: {net_color};"
        )
        prof_info = stats.locked_resp_profile or stats.active_resp_profile or "-"
        self.lbl_net.setToolTip(
            f"61450 Bound: {bound_ip} | Master: {stats.master_ip or '-'}:{stats.master_port} | "
            f"RX={stats.rx_packets} TX={stats.tx_packets} ERR={stats.malformed_packets} | "
            f"Hdr={stats.last_header_len}B Pkt={stats.last_pkt_len}B | 0E70h={prof_info}"
        )

        self.led_csr.set_active(bool(snap["csr"]))
        self.led_ssr.set_active(bool(snap["ssr"]))

        self.lbl_pds.setText(str(snap["pds_state"]))
        self.lbl_mode.setText(str(snap["op_mode"]))
        self.lbl_pos.setText(f"{snap['pos_actual']:,}")
        self.lbl_ferr.setText(f"{snap['following_error']:,}")
        self.lbl_vel.setText(f"{snap['vel_actual']:,}")
        self.lbl_vel.setToolTip(
            f"馬達解析度(608Fh): {snap['encoder_ppr']:,} p/rev | "
            f"電子齒輪比(2006h/2007h): {snap['gear_num']:,} / {snap['gear_den']:,}"
        )
        vlim = int(snap["vel_limit"])
        is_pt = int(snap["op_mode_id"]) == OpMode.PT
        self.lbl_vlim.setText(f"{vlim:,} (Pt有效)" if is_pt else f"{vlim:,} (僅Pt有效)")
        self.lbl_vlim.setStyleSheet(
            f"font-family: Consolas; font-size: 12px; font-weight: bold; color: {'#60A5FA' if is_pt else '#64748B'};"
        )
        self.lbl_cw_sw.setText(f"{snap['controlword']:04X}h / {snap['statusword']:04X}h")
        err_code = int(snap["error_code"])
        self.lbl_err.setText(f"0x{err_code:04X}" if err_code else "0x0000")
        self.lbl_err.setStyleSheet(
            f"font-family: Consolas; font-size: 12px; font-weight: bold; color: {'#EF4444' if err_code else '#64748B'};"
        )

        self.rotor.set_state(float(snap["angle_deg"]), self.axis.pds_state)
        self.slider.set_position(
            int(snap["pos_actual"]),
            bool(snap["lsn"]),
            bool(snap["orgp"]),
            bool(snap["lsp"]),
        )


class NetshScriptDialog(QDialog):
    """一鍵產生 Windows netsh 多靜態 IP 綁定與防火牆放行批次腳本 (.bat) 對話框。"""

    def __init__(self, axes: List[ServoAxis], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.axes = axes
        self.setWindowTitle("Windows 多 IP 綁定與防火牆放行腳本 (netsh)")
        self.resize(740, 460)

        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        top.addWidget(QLabel("Windows 網路介面卡名稱 (如 乙太網路 / Ethernet):"))
        self.edt_iface = QLineEdit("乙太網路")
        self.edt_iface.textChanged.connect(self._regenerate)
        top.addWidget(self.edt_iface, 1)
        layout.addLayout(top)

        self.txt_script = QPlainTextEdit()
        self.txt_script.setReadOnly(True)
        layout.addWidget(self.txt_script, 1)

        btns = QHBoxLayout()
        btn_copy = QPushButton("複製腳本至剪貼簿")
        btn_copy.setObjectName("primaryBtn")
        btn_copy.clicked.connect(self._copy_clipboard)
        btn_save = QPushButton("另存為 .bat 腳本")
        btn_save.clicked.connect(self._save_bat)
        btn_close = QPushButton("關閉")
        btn_close.clicked.connect(self.accept)
        btns.addWidget(btn_copy)
        btns.addWidget(btn_save)
        btns.addStretch()
        btns.addWidget(btn_close)
        layout.addLayout(btns)

        self._regenerate()

    def _regenerate(self) -> None:
        script = generate_windows_netsh_script(self.axes, self.edt_iface.text().strip() or "Ethernet")
        self.txt_script.setPlainText(script)

    def _copy_clipboard(self) -> None:
        QApplication.clipboard().setText(self.txt_script.toPlainText())
        QMessageBox.information(self, "已複製", "已將 netsh 腳本複製到剪貼簿，請以系統管理員身分執行。")

    def _save_bat(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "儲存 Windows 綁定腳本", "setup_sdcf_ips.bat", "Batch Files (*.bat)")
        if path:
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(self.txt_script.toPlainText())
            QMessageBox.information(self, "儲存成功", f"已儲存至：\n{path}")


class NetshRestoreScriptDialog(QDialog):
    """一鍵產生恢復 Windows 原廠網路設定 (DHCP / 移除模擬 IP / 單一固定 IP + 清除防火牆) 批次腳本 (.bat) 對話框。"""

    def __init__(self, axes: List[ServoAxis], parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.axes = axes
        self.setWindowTitle("產生恢復原廠網路腳本 (.bat)")
        self.resize(760, 500)

        layout = QVBoxLayout(self)

        row_iface = QHBoxLayout()
        row_iface.addWidget(QLabel("Windows 網路介面卡名稱 (如 乙太網路 / Ethernet):"))
        self.edt_iface = QLineEdit("乙太網路")
        self.edt_iface.textChanged.connect(self._regenerate)
        row_iface.addWidget(self.edt_iface, 1)
        layout.addLayout(row_iface)

        row_mode = QHBoxLayout()
        row_mode.addWidget(QLabel("恢復方式:"))
        self.cmb_mode = QComboBox()
        self.cmb_mode.addItem("恢復自動取得 IP (DHCP) — 清除全部靜態與多 IP", "dhcp")
        self.cmb_mode.addItem("僅移除本模擬器綁定的多軸 IP（保留原本主 IP）", "delete_sim_ips")
        self.cmb_mode.addItem("覆蓋重設為單一固定 IP", "static")
        self.cmb_mode.currentIndexChanged.connect(self._on_mode_changed)
        row_mode.addWidget(self.cmb_mode, 1)

        self.chk_fw = QCheckBox("一併移除模擬器防火牆規則")
        self.chk_fw.setChecked(True)
        self.chk_fw.stateChanged.connect(self._regenerate)
        row_mode.addWidget(self.chk_fw)
        layout.addLayout(row_mode)

        row_static = QHBoxLayout()
        row_static.addWidget(QLabel("單一固定 IP:"))
        self.edt_static_ip = QLineEdit("192.168.3.250")
        self.edt_static_ip.setFixedWidth(140)
        self.edt_static_ip.textChanged.connect(self._regenerate)
        row_static.addWidget(self.edt_static_ip)
        row_static.addWidget(QLabel("子網路遮罩:"))
        self.edt_static_mask = QLineEdit("255.255.255.0")
        self.edt_static_mask.setFixedWidth(140)
        self.edt_static_mask.textChanged.connect(self._regenerate)
        row_static.addWidget(self.edt_static_mask)
        row_static.addStretch()
        layout.addLayout(row_static)

        self.txt_script = QPlainTextEdit()
        self.txt_script.setReadOnly(True)
        layout.addWidget(self.txt_script, 1)

        btns = QHBoxLayout()
        btn_copy = QPushButton("複製腳本至剪貼簿")
        btn_copy.setObjectName("primaryBtn")
        btn_copy.clicked.connect(self._copy_clipboard)
        btn_save = QPushButton("另存為 .bat 腳本")
        btn_save.clicked.connect(self._save_bat)
        btn_close = QPushButton("關閉")
        btn_close.clicked.connect(self.accept)
        btns.addWidget(btn_copy)
        btns.addWidget(btn_save)
        btns.addStretch()
        btns.addWidget(btn_close)
        layout.addLayout(btns)

        self._on_mode_changed()

    def _on_mode_changed(self) -> None:
        is_static = self.cmb_mode.currentData() == "static"
        self.edt_static_ip.setEnabled(is_static)
        self.edt_static_mask.setEnabled(is_static)
        self._regenerate()

    def _regenerate(self) -> None:
        script = generate_windows_restore_script(
            self.axes,
            interface_name=self.edt_iface.text().strip() or "Ethernet",
            mode=str(self.cmb_mode.currentData() or "dhcp"),
            static_ip=self.edt_static_ip.text().strip() or "192.168.3.250",
            static_mask=self.edt_static_mask.text().strip() or "255.255.255.0",
            remove_firewall=self.chk_fw.isChecked(),
        )
        self.txt_script.setPlainText(script)

    def _copy_clipboard(self) -> None:
        QApplication.clipboard().setText(self.txt_script.toPlainText())
        QMessageBox.information(self, "已複製", "已將恢復網路腳本複製到剪貼簿，請以系統管理員身分執行。")

    def _save_bat(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "儲存 Windows 恢復網路腳本", "restore_sdcf_network.bat", "Batch Files (*.bat)"
        )
        if path:
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(self.txt_script.toPlainText())
            QMessageBox.information(self, "儲存成功", f"已儲存至：\n{path}")


class SDCFSymulatorMainWindow(QMainWindow):
    """士林電機 SDC-F 多軸模擬器主視窗，負責整合物理運動排程、網路伺服器與右側監控面板。"""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("士林電機 SDC-F 多軸伺服驅動器模擬器 (CC-Link IEFB + SLMP)")
        self.resize(1500, 920)

        self.cyclic_cfg = CyclicFrameConfig()
        self.net_manager = MultiAxisCyclicManager(
            self.cyclic_cfg,
            on_network_diag=self._on_network_diag,
        )
        self.bundles: Dict[int, AxisNetworkBundle] = self.net_manager.bundles
        self.cards: Dict[int, AxisCardWidget] = {}
        self.selected_axis_id: int = 1

        self._od_flash_until: Dict[tuple, float] = {}
        self._od_row_map: Dict[tuple, int] = {}
        self._last_od_axis_id: Optional[int] = None
        self._last_od_filter: str = ""

        self._build_ui()

        for ax_id in range(1, 2):
            self._add_axis_internal(ax_id, self._calc_axis_ip(ax_id))
        self._select_axis(1)

        # 10ms 物理引擎步進計時器（更新位置/速度/追隨誤差與看門狗逾時）
        self._last_step_ts = time.perf_counter()
        self.physics_timer = QTimer(self)
        self.physics_timer.timeout.connect(self._on_physics_tick)
        self.physics_timer.start(10)

        # 50ms UI 畫面更新計時器（20 FPS 刷新軸卡片、PDO 與物件字典數值）
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._on_ui_refresh)
        self.ui_timer.start(50)
        QTimer.singleShot(0, self._relayout_axis_cards)

    def _build_ui(self) -> None:
        """建構主畫面整體版面配置（頂部控制列 + 左右可拖曳分割區）。"""
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        main_layout.addWidget(self._build_top_bar())

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.setHandleWidth(6)

        monitor_box = QGroupBox("多軸動態視覺化監控網格")
        monitor_box.setMinimumWidth(460)
        monitor_box.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        monitor_lay = QVBoxLayout(monitor_box)
        monitor_lay.setContentsMargins(6, 10, 6, 6)

        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.grid_container = QWidget()
        self.cards_grid = QGridLayout(self.grid_container)
        self.cards_grid.setSpacing(8)
        self.cards_grid.setContentsMargins(2, 2, 2, 2)
        self.cards_grid.setAlignment(Qt.AlignmentFlag.AlignTop)
        self.scroll_area.setWidget(self.grid_container)
        monitor_lay.addWidget(self.scroll_area)

        right_panel = self._build_right_panel()
        right_panel.setMinimumWidth(520)
        right_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.main_splitter.addWidget(monitor_box)
        self.main_splitter.addWidget(right_panel)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setSizes([780, 700])
        self.main_splitter.splitterMoved.connect(lambda _p, _i: self._relayout_axis_cards())
        main_layout.addWidget(self.main_splitter, 1)

    def _build_top_bar(self) -> QGroupBox:
        """建構頂部精簡控制列（軸數設定、IP 管理、通訊啟停、PA53/PA54 與馬達解析度）。"""
        box = QGroupBox("多軸網路與系統參數控制列 (CC-Link IEFB / PA53 / PA54 / 馬達解析度)")
        vbox = QVBoxLayout(box)
        vbox.setContentsMargins(8, 10, 8, 6)
        vbox.setSpacing(5)

        row1 = QHBoxLayout()
        row1.setSpacing(8)

        row1.addWidget(QLabel("軸數:"))
        self.spn_axis_count = QSpinBox()
        self.spn_axis_count.setRange(1, 8)
        self.spn_axis_count.setValue(1)
        self.spn_axis_count.valueChanged.connect(self._on_axis_count_changed)
        row1.addWidget(self.spn_axis_count)

        row1.addWidget(QLabel("起始 IP:"))
        self.edt_base_ip = QLineEdit("192.168.3.1")
        self.edt_base_ip.setFixedWidth(108)
        self.edt_base_ip.setToolTip("設定第 1 軸起始 IP（後續軸依序遞增 +1）")
        self.edt_base_ip.returnPressed.connect(self._on_apply_base_ip)
        row1.addWidget(self.edt_base_ip)

        btn_apply_base = QPushButton("套用IP")
        btn_apply_base.clicked.connect(self._on_apply_base_ip)
        row1.addWidget(btn_apply_base)

        self.btn_start_net = QPushButton("啟動全部通訊埠")
        self.btn_start_net.setObjectName("successBtn")
        self.btn_start_net.clicked.connect(lambda: asyncio.create_task(self.start_all_servers()))
        row1.addWidget(self.btn_start_net)

        self.btn_stop_net = QPushButton("停止通訊")
        self.btn_stop_net.setObjectName("dangerBtn")
        self.btn_stop_net.clicked.connect(self.stop_all_servers)
        row1.addWidget(self.btn_stop_net)

        btn_netsh = QPushButton("一鍵產生 Windows 多 IP 與防火牆腳本")
        btn_netsh.setObjectName("primaryBtn")
        btn_netsh.clicked.connect(self._show_netsh_dialog)
        row1.addWidget(btn_netsh)

        btn_restore_netsh = QPushButton("產生恢復原廠網路腳本（.bat）")
        btn_restore_netsh.clicked.connect(self._show_restore_netsh_dialog)
        row1.addWidget(btn_restore_netsh)

        row1.addStretch()
        vbox.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(8)

        row2.addWidget(QLabel("PA53 映射:"))
        self.cmb_pa53 = QComboBox()
        self.cmb_pa53.addItem("0: Mode 1 (網路標準模式 1 - 預設)", 0)
        self.cmb_pa53.addItem("1: Mode 2 (標準定位+原點復歸)", 1)
        self.cmb_pa53.addItem("2: Mode 3 (1600h/1A00h 自由映射)", 2)
        self.cmb_pa53.currentIndexChanged.connect(self._on_pa53_changed)
        row2.addWidget(self.cmb_pa53)

        row2.addWidget(QLabel("PA54 單位:"))
        self.cmb_pa54 = QComboBox()
        self.cmb_pa54.addItem("0: 0.01 r/min & ms", 0)
        self.cmb_pa54.addItem("1: pulse/s & pulse/s²", 1)
        self.cmb_pa54.currentIndexChanged.connect(self._on_pa54_changed)
        row2.addWidget(self.cmb_pa54)

        row2.addWidget(QLabel("馬達解析度:"))
        self.cmb_enc_res = QComboBox()
        self.cmb_enc_res.addItem("16777216 (24-bit 預設)", 16777216)
        self.cmb_enc_res.addItem("4194304 (22-bit)", 4194304)
        self.cmb_enc_res.addItem("131072 (17-bit)", 131072)
        self.cmb_enc_res.addItem("10000", 10000)
        self.cmb_enc_res.setToolTip("608Fh:01 馬達編碼器解析度 (pulses/rev)，速度依 解析度 / (2006h/2007h) 換算")
        self.cmb_enc_res.currentIndexChanged.connect(self._on_enc_res_changed)
        row2.addWidget(self.cmb_enc_res)

        row2.addStretch()
        vbox.addLayout(row2)
        return box

    def _build_right_panel(self) -> QTabWidget:
        self.tabs = QTabWidget()

        tab_pdo = QWidget()
        pdo_layout = QVBoxLayout(tab_pdo)
        self.lbl_pdo_header = QLabel("Axis 1 - PA53 = 0 (Mode 1) 循環通訊暫存器即時監控")
        self.lbl_pdo_header.setStyleSheet("font-weight: bold; color: #38BDF8; padding: 2px;")
        pdo_layout.addWidget(self.lbl_pdo_header)

        pdo_split = QSplitter(Qt.Orientation.Vertical)

        rww_box = QGroupBox("Master -> Slave (RWwn00 ~ RWwn1F)")
        rww_lay = QVBoxLayout(rww_box)
        self.tbl_rww = QTableWidget(0, 6)
        self.tbl_rww.setHorizontalHeaderLabels(["Word", "Byte", "Object", "Type", "Name", "Hex / Dec"])
        for c in (0, 1, 2, 3, 5):
            self.tbl_rww.horizontalHeader().setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        self.tbl_rww.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.tbl_rww.verticalHeader().setVisible(False)
        self.tbl_rww.setAlternatingRowColors(True)
        rww_lay.addWidget(self.tbl_rww)
        pdo_split.addWidget(rww_box)

        rwr_box = QGroupBox("Slave -> Master (RWrn00 ~ RWrn1F)")
        rwr_lay = QVBoxLayout(rwr_box)
        self.tbl_rwr = QTableWidget(0, 6)
        self.tbl_rwr.setHorizontalHeaderLabels(["Word", "Byte", "Object", "Type", "Name", "Hex / Dec"])
        for c in (0, 1, 2, 3, 5):
            self.tbl_rwr.horizontalHeader().setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        self.tbl_rwr.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeMode.Stretch)
        self.tbl_rwr.verticalHeader().setVisible(False)
        self.tbl_rwr.setAlternatingRowColors(True)
        rwr_lay.addWidget(self.tbl_rwr)
        pdo_split.addWidget(rwr_box)

        pdo_layout.addWidget(pdo_split, 1)
        self.tabs.addTab(tab_pdo, "PDO 監控 (RWwn / RWrn)")

        tab_od = QWidget()
        od_lay = QVBoxLayout(tab_od)
        od_filter_row = QHBoxLayout()
        od_filter_row.addWidget(QLabel("篩選物件 (Index / 名稱):"))
        self.edt_od_filter = QLineEdit()
        self.edt_od_filter.setPlaceholderText("例如: 6098, 6099, 2D20, PA53, Homing...")
        self.edt_od_filter.textChanged.connect(lambda: self._populate_od_table(force_rebuild=True))
        od_filter_row.addWidget(self.edt_od_filter, 1)
        od_lay.addLayout(od_filter_row)

        self.tbl_od = QTableWidget(0, 8)
        self.tbl_od.setHorizontalHeaderLabels(
            ["Index:Sub", "Object Name", "Type", "Access", "Dec Value", "Hex Value", "Unit", "Note"]
        )
        self.tbl_od.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.tbl_od.verticalHeader().setVisible(False)
        self.tbl_od.setAlternatingRowColors(True)
        self.tbl_od.cellDoubleClicked.connect(self._on_od_double_clicked)
        od_lay.addWidget(self.tbl_od, 1)
        self.tabs.addTab(tab_od, "SLMP 物件字典 (1000h~6FFFh)")

        tab_log = QWidget()
        log_lay = QVBoxLayout(tab_log)
        log_hdr = QHBoxLayout()
        log_hdr.addWidget(QLabel("SLMP 4020h / 0E30h 與伺服狀態事件日誌"))
        btn_clr_log = QPushButton("清除日誌")
        btn_clr_log.clicked.connect(lambda: self.txt_log.clear())
        log_hdr.addStretch()
        log_hdr.addWidget(btn_clr_log)
        log_lay.addLayout(log_hdr)

        self.txt_log = QPlainTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumBlockCount(500)
        log_lay.addWidget(self.txt_log, 1)
        self.tabs.addTab(tab_log, "SLMP 通訊日誌")

        # 將 SLMP 4020h 測試發送器納入第 4 分頁，讓主監控畫面更簡潔寬敞
        self.tabs.addTab(self._build_slmp_tester_tab(), "SLMP 4020h 測試")

        return self.tabs

    def _build_slmp_tester_tab(self) -> QWidget:
        """建構右側第 4 分頁：內建 SLMP 4020h (UDP Port 5010) 讀寫測試發送器。"""
        tab = QWidget()
        outer = QVBoxLayout(tab)
        outer.setContentsMargins(8, 10, 8, 8)
        box = QGroupBox("內建 SLMP 4020h 封包測試發送器 (UDP Port 5010 - 3E Binary)")
        lay = QVBoxLayout(box)
        lay.setContentsMargins(8, 10, 8, 8)
        lay.setSpacing(8)

        row1 = QHBoxLayout()
        row1.setSpacing(8)
        row1.addWidget(QLabel("目標軸 IP:"))
        self.cmb_test_ip = QComboBox()
        self.cmb_test_ip.setEditable(True)
        self.cmb_test_ip.setFixedWidth(130)
        row1.addWidget(self.cmb_test_ip)

        row1.addWidget(QLabel("指令 (4020h):"))
        self.cmb_test_subcmd = QComboBox()
        self.cmb_test_subcmd.addItem("0001h Read Object", SLMP_SUB_READ_OBJ)
        self.cmb_test_subcmd.addItem("0002h Write Object", SLMP_SUB_WRITE_OBJ)
        self.cmb_test_subcmd.addItem("0005h SubID Block Read", SLMP_SUB_READ_BLOCK)
        self.cmb_test_subcmd.addItem("0006h SubID Block Write", SLMP_SUB_WRITE_BLOCK)
        row1.addWidget(self.cmb_test_subcmd)

        row1.addWidget(QLabel("Index (Hex):"))
        self.edt_test_idx = QLineEdit("6098")
        self.edt_test_idx.setFixedWidth(65)
        row1.addWidget(self.edt_test_idx)

        row1.addWidget(QLabel("SubIndex (Hex):"))
        self.edt_test_sub = QLineEdit("00")
        self.edt_test_sub.setFixedWidth(45)
        row1.addWidget(self.edt_test_sub)

        row1.addWidget(QLabel("Block Count:"))
        self.spn_test_cnt = QSpinBox()
        self.spn_test_cnt.setRange(1, 32)
        self.spn_test_cnt.setValue(1)
        row1.addWidget(self.spn_test_cnt)
        row1.addStretch()
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.setSpacing(8)
        row2.addWidget(QLabel("資料型態:"))
        self.cmb_test_dtype = QComboBox()
        self.cmb_test_dtype.addItem("SINT (1B)", ("SINT", 1, True))
        self.cmb_test_dtype.addItem("USINT (1B)", ("USINT", 1, False))
        self.cmb_test_dtype.addItem("INT (2B)", ("INT", 2, True))
        self.cmb_test_dtype.addItem("UINT (2B)", ("UINT", 2, False))
        self.cmb_test_dtype.addItem("DINT (4B)", ("DINT", 4, True))
        self.cmb_test_dtype.addItem("UDINT (4B)", ("UDINT", 4, False))
        self.cmb_test_dtype.addItem("Raw Hex", ("HEX", 0, False))
        row2.addWidget(self.cmb_test_dtype)

        row2.addWidget(QLabel("Write Value:"))
        self.edt_test_val = QLineEdit("19")
        self.edt_test_val.setFixedWidth(150)
        row2.addWidget(self.edt_test_val)

        btn_send = QPushButton("發送 SLMP 3E 封包")
        btn_send.setObjectName("primaryBtn")
        btn_send.clicked.connect(lambda: asyncio.create_task(self._on_send_slmp_test()))
        row2.addWidget(btn_send)
        row2.addStretch()
        lay.addLayout(row2)

        self.lbl_test_result = QLabel("就緒：可發送真實 UDP 封包至 Port 5010 測試 6098h 原點復歸方式或 6099h 速度讀寫。")
        self.lbl_test_result.setWordWrap(True)
        self.lbl_test_result.setStyleSheet("color: #94A3B8; font-family: Consolas; font-size: 11px;")
        lay.addWidget(self.lbl_test_result)

        outer.addWidget(box)
        outer.addStretch()
        return tab

    def _calc_axis_ip(self, axis_id: int) -> str:
        """依起始 IP 與軸編號推算預設 IPv4 位址（例如 192.168.3.1 + (axis_id - 1)）。"""
        base_str = self.edt_base_ip.text().strip() if hasattr(self, "edt_base_ip") else "192.168.3.1"
        try:
            base_ip = ipaddress.IPv4Address(base_str)
            return str(base_ip + (axis_id - 1))
        except ipaddress.AddressValueError:
            return f"192.168.3.{axis_id}"

    def _add_axis_internal(self, axis_id: int, ip_address: str) -> None:
        """建立單一模擬軸與其網路伺服綁定，並同步目前 UI 設定之解析度與 PA54 單位。"""
        axis = ServoAxis(axis_id=axis_id, ip_address=ip_address)
        if hasattr(self, "cmb_enc_res"):
            axis.od.set_internal(0x608F, 0x01, int(self.cmb_enc_res.currentData() or ENCODER_PPR))
        if hasattr(self, "cmb_pa54"):
            axis.od.set_internal(0x2036, 0x00, int(self.cmb_pa54.currentData() or 0))
        axis.on_event = self._on_axis_event
        bundle = self.net_manager.add_axis(
            axis=axis,
            on_slmp_transaction=self._on_slmp_transaction,
            on_object_modified=self._on_slmp_object_modified,
            on_ip_change=self._on_ip_changed,
            on_network_diag=self._on_network_diag,
        )

        card = AxisCardWidget(bundle)
        card.selected.connect(self._select_axis)
        card.ip_change_requested.connect(self._on_axis_ip_change_requested)
        self.cards[axis_id] = card

        self._relayout_axis_cards()
        self._refresh_test_ip_combo()

    def _relayout_axis_cards(self) -> None:
        if not hasattr(self, "cards_grid"):
            return
        for c in list(self.cards.values()):
            self.cards_grid.removeWidget(c)

        avail_w = self.scroll_area.viewport().width() if hasattr(self, "scroll_area") else 780
        cols = 2 if (len(self.cards) >= 2 and avail_w >= 920) else 1

        for col in range(2):
            self.cards_grid.setColumnStretch(col, 1 if col < cols else 0)

        for idx, ax_id in enumerate(sorted(self.cards.keys())):
            card = self.cards[ax_id]
            r = idx // cols
            c = idx % cols
            self.cards_grid.addWidget(card, r, c)

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self._relayout_axis_cards()

    def _on_apply_base_ip(self) -> None:
        base_str = self.edt_base_ip.text().strip()
        try:
            base_ip = ipaddress.IPv4Address(base_str)
        except ipaddress.AddressValueError:
            QMessageBox.warning(self, "IP 格式錯誤", f"無效的 IPv4 位址：{base_str}")
            return
        for ax_id in sorted(self.bundles.keys()):
            new_ip = str(base_ip + (ax_id - 1))
            self._update_axis_ip(ax_id, new_ip)

    def _on_axis_ip_change_requested(self, axis_id: int, new_ip: str) -> None:
        try:
            ipaddress.IPv4Address(new_ip)
        except ipaddress.AddressValueError:
            QMessageBox.warning(self, "IP 格式錯誤", f"無效的 IPv4 位址：{new_ip}")
            if axis_id in self.bundles and axis_id in self.cards:
                self.cards[axis_id].edt_ip.setText(self.bundles[axis_id].axis.ip_address)
            return
        self._update_axis_ip(axis_id, new_ip)

    def _update_axis_ip(self, axis_id: int, new_ip: str) -> None:
        bundle = self.bundles.get(axis_id)
        if bundle is None or bundle.axis.ip_address == new_ip:
            return
        was_running = bundle.cyclic_server.is_running or bundle.slmp_server.is_running
        bundle.axis.ip_address = new_ip
        if axis_id in self.cards:
            self.cards[axis_id].edt_ip.setText(new_ip)
        self._refresh_test_ip_combo()
        if axis_id == self.selected_axis_id:
            self.cmb_test_ip.setCurrentText(new_ip)
        self._append_log(f"[Network] Axis {axis_id} IP 已修改為 {new_ip}")
        if was_running:
            bundle.stop_all()
            asyncio.create_task(self._start_single_bundle(bundle))

    def _on_axis_count_changed(self, target_count: int) -> None:
        current_count = len(self.bundles)
        if target_count > current_count:
            for ax_id in range(current_count + 1, target_count + 1):
                self._add_axis_internal(ax_id, self._calc_axis_ip(ax_id))
                if any(b.all_running for b in self.bundles.values()):
                    asyncio.create_task(self._start_single_bundle(self.bundles[ax_id]))
        elif target_count < current_count:
            for ax_id in range(current_count, target_count, -1):
                self.net_manager.remove_axis(ax_id)
                card = self.cards.pop(ax_id)
                self.cards_grid.removeWidget(card)
                card.deleteLater()
            if self.selected_axis_id not in self.bundles:
                self._select_axis(1)
            self._relayout_axis_cards()
            self._refresh_test_ip_combo()

    def _refresh_test_ip_combo(self) -> None:
        cur = self.cmb_test_ip.currentText()
        self.cmb_test_ip.blockSignals(True)
        self.cmb_test_ip.clear()
        for ax_id in sorted(self.bundles.keys()):
            self.cmb_test_ip.addItem(self.bundles[ax_id].axis.ip_address, ax_id)
        if cur:
            self.cmb_test_ip.setCurrentText(cur)
        self.cmb_test_ip.blockSignals(False)

    async def _start_single_bundle(self, bundle: AxisNetworkBundle) -> None:
        results = await bundle.start_all()
        for label, ok, msg in results:
            self._append_log(f"[Axis {bundle.axis.axis_id}] {label}: {'OK' if ok else 'ERR'} - {msg}")
        if bundle.cyclic_server.bound_ip == "127.0.0.1":
            self._append_log(
                f"[警告 Axis {bundle.axis.axis_id}] 61450 僅綁定到 127.0.0.1 回環位址，"
                "外部 AX5S PLC 封包將無法送達！請確認網卡 IP 或點擊「一鍵產生 Windows 多 IP 與防火牆腳本」。"
            )
        bcast_msgs = await self.net_manager.start_broadcast_listeners()
        for bm in bcast_msgs:
            self._append_log(f"[Broadcast] {bm}")

    async def start_all_servers(self) -> None:
        self._append_log("=== 正在啟動所有模擬軸通訊伺服器 (UDP 61450/61451/5010/45237 + TCP 5010) ===")
        self.net_manager.stop_all()
        report = await self.net_manager.start_all()
        for ax_id in sorted(report.keys()):
            for label, ok, msg in report[ax_id]:
                self._append_log(f"[Axis {ax_id}] {label}: {'OK' if ok else 'ERR'} - {msg}")
            bundle = self.bundles.get(ax_id)
            if bundle is not None and bundle.cyclic_server.bound_ip == "127.0.0.1":
                self._append_log(
                    f"[警告 Axis {ax_id}] 61450 僅綁定到 127.0.0.1，外部 PLC 無法連線！"
                )
        bcast_msgs = await self.net_manager.start_broadcast_listeners()
        for bm in bcast_msgs:
            self._append_log(f"[Broadcast] {bm}")
        for dm in diagnose_local_network([self.bundles[ax_id].axis for ax_id in sorted(self.bundles.keys())]):
            self._append_log(dm)

    def stop_all_servers(self) -> None:
        self.net_manager.stop_all()
        self._append_log("=== 已停止所有模擬軸 UDP 伺服器 ===")
        self._on_ui_refresh()

    def _select_axis(self, axis_id: int) -> None:
        """切換目前選中監控之軸，更新卡片外框高亮與物件字典內容。"""
        self.selected_axis_id = axis_id
        for cid, card in self.cards.items():
            card.set_selected(cid == axis_id)
        if axis_id in self.bundles:
            self.cmb_test_ip.setCurrentText(self.bundles[axis_id].axis.ip_address)
        self._populate_od_table(force_rebuild=True)

    def _on_pa53_changed(self) -> None:
        """套用全部軸之 PA53 (2035h) PDO 映射模式（Mode 1 / Mode 2 / Mode 3）。"""
        mode = int(self.cmb_pa53.currentData())
        for bundle in self.bundles.values():
            bundle.axis.od.apply_mapping_mode(mode)
        self._append_log(f"[System] 全部軸已切換 PA53 映射模式 = {mode} ({self.cmb_pa53.currentText()})")
        self._populate_od_table(force_rebuild=True)

    def _on_pa54_changed(self) -> None:
        """套用全部軸之 PA54 (2036h) 通訊速度/加減速單位設定。"""
        unit_sel = int(self.cmb_pa54.currentData())
        for bundle in self.bundles.values():
            bundle.axis.od.set_internal(0x2036, 0x00, unit_sel)
        self._append_log(f"[System] 全部軸已切換 PA54 單位 = {unit_sel} ({self.cmb_pa54.currentText()})")

    def _on_enc_res_changed(self) -> None:
        """套用全部軸之馬達編碼器解析度 (608Fh:01)。"""
        res = int(self.cmb_enc_res.currentData() or ENCODER_PPR)
        for bundle in self.bundles.values():
            bundle.axis.od.set_internal(0x608F, 0x01, res)
        self._append_log(f"[System] 全部軸已切換馬達解析度 (608Fh:01) = {res:,} pulse/rev")
        self._populate_od_table(force_rebuild=True)

    def _show_netsh_dialog(self) -> None:
        axes = [self.bundles[k].axis for k in sorted(self.bundles.keys())]
        dlg = NetshScriptDialog(axes, self)
        dlg.exec()

    def _show_restore_netsh_dialog(self) -> None:
        axes = [self.bundles[k].axis for k in sorted(self.bundles.keys())]
        dlg = NetshRestoreScriptDialog(axes, self)
        dlg.exec()

    def _on_axis_event(self, axis_id: int, msg: str) -> None:
        self._append_log(f"[Axis {axis_id} Motion] {msg}")

    def _on_network_diag(self, axis_id: int, msg: str) -> None:
        self._append_log(f"[Axis {axis_id} Net] {msg}")

    def _on_slmp_transaction(self, tx: SLMPTransaction) -> None:
        err_str = SLMP_ERROR_MESSAGES.get(tx.end_code, f"0x{tx.end_code:04X}")
        self._append_log(
            f"[SLMP {tx.frame_type} -> Axis {tx.axis_id}] From {tx.peer_ip}:{tx.peer_port} | "
            f"Cmd={tx.command:04X}h Sub={tx.subcommand:04X}h | {tx.summary} | EndCode=0x{tx.end_code:04X} ({err_str})"
        )

    def _on_slmp_object_modified(self, axis_id: int, index: int, subindex: int, hex_data: str) -> None:
        self._od_flash_until[(axis_id, index, subindex)] = time.time() + 2.5
        if axis_id == self.selected_axis_id:
            row = self._od_row_map.get((axis_id, index, subindex))
            if row is not None:
                self.tbl_od.scrollToItem(self.tbl_od.item(row, 0))

    def _on_ip_changed(self, axis_id: int, new_ip: str, new_mask: str, new_gw: str) -> None:
        self._append_log(f"[NodeSearch/IPSet] Axis {axis_id} IP 變更為 {new_ip} ({new_mask})")
        self._refresh_test_ip_combo()

    def _append_log(self, line: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.txt_log.appendPlainText(f"[{ts}] {line}")

    def _on_physics_tick(self) -> None:
        """每 10ms 推進各軸物理運動模型並檢查循環通訊看門狗是否逾時。"""
        now = time.perf_counter()
        dt = now - self._last_step_ts
        self._last_step_ts = now
        for bundle in self.bundles.values():
            bundle.axis.step(dt)
            bundle.cyclic_server.check_watchdog()

    def _on_ui_refresh(self) -> None:
        """每 50ms 定期刷新所有軸卡片、PDO 監控表與物件字典顯示數值。"""
        for card in self.cards.values():
            card.refresh()
        self._refresh_pdo_tables()
        self._refresh_od_values()

    def _refresh_pdo_tables(self) -> None:
        """依目前選中軸的 PDO 映射更新右側 RWwn / RWrn 監控表格。"""
        bundle = self.bundles.get(self.selected_axis_id)
        if bundle is None:
            return
        axis = bundle.axis
        pa53 = axis.od.get_int(0x2035, 0x00)
        mode_name = {0: "Mode 1 (網路標準模式 1)", 1: "Mode 2 (標準定位+原點)", 2: "Mode 3 (自由映射)"}.get(pa53, str(pa53))
        ry_hex = axis.last_ry_bytes.hex(" ").upper()
        rx_hex = axis.last_rx_bytes.hex(" ").upper()
        self.lbl_pdo_header.setText(
            f"Axis {axis.axis_id} ({axis.ip_address}) - PA53 = {pa53}: {mode_name} | "
            f"CSR={int(axis.csr_active)} SSR={int(axis.ssr_active)}\n"
            f"RY(64b): {ry_hex}   |   RX(64b): {rx_hex}"
        )

        rww_rows = axis.od.get_pdo_word_rows(is_rx=True, raw_bytes=axis.last_rww_bytes)
        rwr_rows = axis.od.get_pdo_word_rows(is_rx=False, raw_bytes=axis.last_rwr_bytes)
        self._fill_pdo_table(self.tbl_rww, rww_rows)
        self._fill_pdo_table(self.tbl_rwr, rwr_rows)

    @staticmethod
    def _fill_pdo_table(table: QTableWidget, rows: List[Dict[str, object]]) -> None:
        if table.rowCount() != len(rows):
            table.setRowCount(len(rows))
        for r_idx, row in enumerate(rows):
            is_res = bool(row["is_reserved"])
            fg = QColor("#64748B") if is_res else QColor("#E2E8F0")
            val_text = f"{row['hex_val']}  ({row['dec_val']})"
            vals = [
                str(row["word_label"]),
                str(row["byte_range"]),
                str(row["object"]),
                str(row["data_type"]),
                str(row["name"]),
                val_text,
            ]
            for c_idx, txt in enumerate(vals):
                item = table.item(r_idx, c_idx)
                if item is None:
                    item = QTableWidgetItem(txt)
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    table.setItem(r_idx, c_idx, item)
                elif item.text() != txt:
                    item.setText(txt)
                item.setForeground(QBrush(QColor("#38BDF8") if (c_idx == 5 and not is_res) else fg))

    def _populate_od_table(self, force_rebuild: bool = False) -> None:
        bundle = self.bundles.get(self.selected_axis_id)
        if bundle is None:
            return
        flt = self.edt_od_filter.text().strip().lower()
        if not force_rebuild and self._last_od_axis_id == self.selected_axis_id and self._last_od_filter == flt:
            return

        self._last_od_axis_id = self.selected_axis_id
        self._last_od_filter = flt
        self._od_row_map.clear()

        entries = []
        for entry in bundle.axis.od.list_all_entries():
            key_str = f"{entry.index:04x}h:{entry.subindex:02x}"
            if flt and (flt not in key_str) and (flt not in entry.name.lower()) and (flt not in entry.description.lower()):
                continue
            entries.append(entry)

        self.tbl_od.setRowCount(len(entries))
        for r_idx, entry in enumerate(entries):
            self._od_row_map[(self.selected_axis_id, entry.index, entry.subindex)] = r_idx
            hex_s = self._format_entry_hex(entry)
            cells = [
                f"{entry.index:04X}h:{entry.subindex:02X}",
                entry.name,
                entry.data_type.value,
                entry.access.value,
                str(entry.value),
                hex_s,
                entry.unit,
                entry.description,
            ]
            for c_idx, txt in enumerate(cells):
                item = QTableWidgetItem(txt)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                if c_idx == 3:
                    item.setForeground(QBrush(QColor("#10B981") if entry.access == AccessType.RW else QColor("#94A3B8")))
                self.tbl_od.setItem(r_idx, c_idx, item)

    def _refresh_od_values(self) -> None:
        bundle = self.bundles.get(self.selected_axis_id)
        if bundle is None:
            return
        now = time.time()
        od = bundle.axis.od
        for (ax_id, idx, sub), r_idx in self._od_row_map.items():
            if ax_id != self.selected_axis_id:
                continue
            entry = od.entries.get((idx, sub))
            if entry is None:
                continue
            dec_item = self.tbl_od.item(r_idx, 4)
            hex_item = self.tbl_od.item(r_idx, 5)
            dec_s = str(entry.value)
            hex_s = self._format_entry_hex(entry)
            if dec_item and dec_item.text() != dec_s:
                dec_item.setText(dec_s)
            if hex_item and hex_item.text() != hex_s:
                hex_item.setText(hex_s)

            expire = self._od_flash_until.get((ax_id, idx, sub), 0.0)
            is_flashing = expire > now
            bg = QBrush(QColor("#78350F")) if is_flashing else QBrush(Qt.BrushStyle.NoBrush)
            for c_idx in range(self.tbl_od.columnCount()):
                cell = self.tbl_od.item(r_idx, c_idx)
                if cell is not None:
                    cell.setBackground(bg)

    @staticmethod
    def _format_entry_hex(entry) -> str:
        if entry.data_type == DataType.STRING:
            return str(entry.value).encode("ascii", errors="replace").hex(" ").upper()
        mask = (1 << (entry.byte_size * 8)) - 1
        return f"0x{int(entry.value) & mask:0{entry.byte_size * 2}X}"

    def _on_od_double_clicked(self, row: int, col: int) -> None:
        bundle = self.bundles.get(self.selected_axis_id)
        if bundle is None:
            return
        key_item = self.tbl_od.item(row, 0)
        if key_item is None:
            return
        idx_str, sub_str = key_item.text().split(":")
        idx = int(idx_str.replace("h", ""), 16)
        sub = int(sub_str, 16)
        entry = bundle.axis.od.entries.get((idx, sub))
        if entry is None:
            return

        self.edt_test_idx.setText(f"{idx:04X}")
        self.edt_test_sub.setText(f"{sub:02X}")
        self.edt_test_val.setText(str(entry.value))
        dtype_map = {
            DataType.SINT: 0,
            DataType.USINT: 1,
            DataType.INT: 2,
            DataType.UINT: 3,
            DataType.DINT: 4,
            DataType.UDINT: 5,
        }
        if entry.data_type in dtype_map:
            self.cmb_test_dtype.setCurrentIndex(dtype_map[entry.data_type])

        if entry.access == AccessType.RO:
            return

        dlg = QDialog(self)
        dlg.setWindowTitle(f"修改物件 {idx:04X}h:{sub:02X} ({entry.name})")
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel(f"型態: {entry.data_type.value} | 範圍: {entry.min_val} ~ {entry.max_val}"))
        edt = QLineEdit(str(entry.value))
        lay.addWidget(edt)
        btns = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lay.addWidget(btns)

        if dlg.exec() == QDialog.DialogCode.Accepted:
            try:
                raw_txt = edt.text().strip()
                new_val = int(raw_txt, 0)
                raw_bytes = struct.pack(entry.data_type.struct_fmt, new_val)
                err = bundle.axis.od.slmp_write_object(idx, sub, len(raw_bytes), raw_bytes, source="GUI Edit")
                if err != SLMPError.OK:
                    QMessageBox.warning(self, "寫入錯誤", f"SLMP EndCode: 0x{err:04X}\n{SLMP_ERROR_MESSAGES.get(err, '')}")
                else:
                    self._od_flash_until[(self.selected_axis_id, idx, sub)] = time.time() + 1.5
                    self._append_log(f"[GUI Edit -> Axis {self.selected_axis_id}] {idx:04X}h:{sub:02X} <= {new_val}")
            except Exception as exc:
                QMessageBox.warning(self, "格式錯誤", str(exc))

    async def _on_send_slmp_test(self) -> None:
        """從「SLMP 4020h 測試」分頁組裝並送出真實 UDP Port 5010 3E Binary 測試封包。"""
        target_ip = self.cmb_test_ip.currentText().strip()
        subcmd = int(self.cmb_test_subcmd.currentData())
        try:
            index = int(self.edt_test_idx.text().strip().replace("0x", "").replace("h", ""), 16)
            subindex = int(self.edt_test_sub.text().strip().replace("0x", ""), 16)
        except ValueError:
            self.lbl_test_result.setText("錯誤：Index 或 SubIndex 必須為有效的 Hex 數值。")
            self.lbl_test_result.setStyleSheet("color: #EF4444; font-family: Consolas;")
            return

        send_ip = target_ip
        for b in self.bundles.values():
            if b.axis.ip_address == target_ip and b.slmp_server.is_running:
                if b.slmp_server.bound_ip not in ("0.0.0.0", target_ip):
                    send_ip = b.slmp_server.bound_ip
                break

        write_bytes = b""
        block_cnt = self.spn_test_cnt.value()
        if subcmd in (SLMP_SUB_WRITE_OBJ, SLMP_SUB_WRITE_BLOCK):
            dtype_name, byte_sz, is_signed = self.cmb_test_dtype.currentData()
            val_txt = self.edt_test_val.text().strip()
            try:
                if dtype_name == "HEX":
                    write_bytes = bytes.fromhex(val_txt.replace("0x", "").replace(" ", ""))
                elif subcmd == SLMP_SUB_WRITE_OBJ:
                    val_int = int(val_txt, 0)
                    write_bytes = SLMPClientTester.encode_integer_le(val_int, byte_sz, is_signed)
                else:
                    if "," in val_txt:
                        parts = [int(p.strip(), 0) for p in val_txt.split(",")]
                        write_bytes = b"".join(
                            SLMPClientTester.encode_integer_le(p, byte_sz, is_signed) for p in parts
                        )
                    else:
                        write_bytes = bytes.fromhex(val_txt.replace("0x", "").replace(" ", ""))
            except Exception as exc:
                self.lbl_test_result.setText(f"數值編碼失敗: {exc}")
                self.lbl_test_result.setStyleSheet("color: #EF4444; font-family: Consolas;")
                return

        resp = await SLMPClientTester.send_request_async(
            target_ip=send_ip,
            subcommand=subcmd,
            index=index,
            subindex=subindex,
            write_data=write_bytes,
            block_count=block_cnt,
        )

        tx_hex = resp.raw_tx.hex(" ").upper()
        rx_hex = resp.raw_rx.hex(" ").upper()
        if resp.ok:
            extra = f" | Data={resp.data_bytes.hex(' ').upper()}" if resp.data_bytes else ""
            if resp.decoded_value is not None:
                extra += f" (Dec={resp.decoded_value})"
            self.lbl_test_result.setText(
                f"[OK 0x0000] {index:04X}h:{subindex:02X}{extra}  |  TX: {tx_hex}  |  RX: {rx_hex}"
            )
            self.lbl_test_result.setStyleSheet("color: #10B981; font-family: Consolas; font-size: 11px;")
        else:
            self.lbl_test_result.setText(
                f"[ERR 0x{resp.end_code:04X}] {resp.error_message}  |  TX: {tx_hex}"
            )
            self.lbl_test_result.setStyleSheet("color: #EF4444; font-family: Consolas; font-size: 11px;")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.physics_timer.stop()
        self.ui_timer.stop()
        self.stop_all_servers()
        super().closeEvent(event)


def main() -> None:
    """程式進入點：建立 PySide6 應用程式與 qasync 非同步事件迴圈並啟動多軸模擬伺服器。"""
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_INDUSTRIAL_QSS)

    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = SDCFSymulatorMainWindow()
    window.show()
    loop.create_task(window.start_all_servers())

    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()
