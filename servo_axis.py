from __future__ import annotations

import math
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Dict, Optional, Tuple

from object_dictionary import ENCODER_PPR, ObjectDictionary, RATED_SPEED_RPM


class PDSState(IntEnum):
    NOT_READY_TO_SWITCH_ON = 0
    SWITCH_ON_DISABLED = 1
    READY_TO_SWITCH_ON = 2
    SWITCHED_ON = 3
    OPERATION_ENABLED = 4
    QUICK_STOP_ACTIVE = 5
    FAULT_REACTION_ACTIVE = 6
    FAULT = 7

    @property
    def label(self) -> str:
        names = {
            PDSState.NOT_READY_TO_SWITCH_ON: "Not Ready to Switch On",
            PDSState.SWITCH_ON_DISABLED: "Switch On Disabled",
            PDSState.READY_TO_SWITCH_ON: "Ready to Switch On",
            PDSState.SWITCHED_ON: "Switched On",
            PDSState.OPERATION_ENABLED: "Operation Enabled (Servo ON)",
            PDSState.QUICK_STOP_ACTIVE: "Quick Stop Active",
            PDSState.FAULT_REACTION_ACTIVE: "Fault Reaction Active",
            PDSState.FAULT: "Fault",
        }
        return names[self]


class OpMode(IntEnum):
    NO_MODE = 0
    PP = 1   # Profile Position
    PV = 3   # Profile Velocity
    PT = 4   # Profile Torque
    HM = 6   # Homing

    @property
    def label(self) -> str:
        names = {
            OpMode.NO_MODE: "0: No Mode",
            OpMode.PP: "1: Profile Position (PP)",
            OpMode.PV: "3: Profile Velocity (PV)",
            OpMode.PT: "4: Profile Torque (PT)",
            OpMode.HM: "6: Homing (HM)",
        }
        return names.get(self, f"{int(self)}: Unknown")


class HomingPhase(IntEnum):
    IDLE = 0
    SEARCH_SWITCH = 1
    SEARCH_ZERO = 2
    MOVE_OFFSET = 3
    DONE = 4
    ERROR = 5


@dataclass
class AxisSensors:
    lsp: bool = False       # Positive Limit Switch (LSP)
    lsn: bool = False       # Negative Limit Switch (LSN)
    orgp: bool = False      # Home / Origin Dog Switch (ORGP)
    z_phase: bool = False   # Encoder Z-phase pulse
    virtual_sensors_enabled: bool = True
    virtual_lsn_pos: int = -1000000
    virtual_lsp_pos: int = 1000000
    virtual_orgp_min: int = -5000
    virtual_orgp_max: int = 15000


class ServoAxis:
    """
    士林電機 SDC-F 單軸伺服模擬引擎。
    實作 CiA 402 PDS 狀態機、PP / HM / PV / PT 四種運轉模式、
    60F4h 追隨誤差與 PC05 誤差過大保護、2D20h Pt 專用限速，
    以及與實機 SDC-F 一致之 CC-Link IEFB RY3F / RX3F+RX3E (0xC0) 握手。
    """

    def __init__(
        self,
        axis_id: int,
        ip_address: str,
        subnet_mask: str = "255.255.255.0",
        gateway: str = "192.168.3.1",
        station_no: Optional[int] = None,
    ) -> None:
        self.axis_id = axis_id
        self.station_no = station_no if station_no is not None else axis_id
        self.ip_address = ip_address
        self.subnet_mask = subnet_mask
        self.gateway = gateway
        self.mac_address = bytes([0x00, 0x1E, 0x08, 0x5D, 0xCF, axis_id & 0xFF])

        self.od = ObjectDictionary(axis_id=axis_id)
        self.sensors = AxisSensors()

        self.csr_active: bool = False  # 主站來訊旗標 RY(n+3)F (CSR)
        self.ssr_active: bool = False  # 從站就緒旗標 RX(n+3)F (SSR)
        self.last_rx_bytes: bytes = b"\x00" * 8
        self.last_ry_bytes: bytes = b"\x00" * 8
        self.last_rww_bytes: bytes = b"\x00" * 64
        self.last_rwr_bytes: bytes = self.od.pack_tx_pdo()
        self.cyclic_rx_count: int = 0
        self.cyclic_tx_count: int = 0
        self.last_cyclic_time: float = 0.0

        self.pds_state: PDSState = PDSState.SWITCH_ON_DISABLED
        self._prev_cw: int = 0
        self._prev_pds_cw: int = 0
        self._prev_op_mode: int = self.od.get_int(0x6061, 0x00)

        self._pos_demand_f: float = 0.0
        self._pos_actual_f: float = 0.0
        self._vel_demand_pps: float = 0.0
        self._vel_actual_pps: float = 0.0
        self._torque_actual_01pct: float = 0.0
        self._motor_angle_deg: float = 0.0

        self._pp_target_pos: int = 0
        self._pp_active: bool = False
        self._pp_in_pos_timer: float = 0.0
        self._pp_target_reached: bool = True
        self._pp_sp_ack: bool = False
        self._pp_sp_latched_in_cyclic: bool = False

        self._hm_phase: HomingPhase = HomingPhase.IDLE
        self._hm_attained: bool = False
        self._hm_error: bool = False
        self._hm_search_dir: int = 1
        self._hm_prev_orgp: bool = False
        self._hm_prev_lsn: bool = False
        self._hm_prev_lsp: bool = False
        self._hm_prev_rev: int = 0
        self._hm_switch_hit: bool = False
        self._hm_start_latched_in_cyclic: bool = False
        self._hm_waiting_orgp_fall: bool = False
        self._hm_need_leave_orgp: bool = False
        self._hm_saw_orgp_wrong_side: bool = False
        self._hm_slow_approach: bool = False

        self._fault_code: int = 0x0000
        self._fault_name: str = ""
        self._torque_limiting: bool = False
        self._vel_limiting: bool = False

        self.on_event: Optional[Callable[[int, str], None]] = None

        self._update_status_objects()

    def _encoder_ppr(self) -> float:
        inc = max(1, self.od.get_int(0x608F, 0x01))
        revs = max(1, self.od.get_int(0x608F, 0x02))
        return float(inc) / float(revs)

    def _gear_ratio(self) -> float:
        """Electronic gear ratio CMX/CDV = 2006h/2007h (encoder pulses per command pulse)."""
        cmx = max(1, self.od.get_int(0x2006, 0x00))
        cdv = max(1, self.od.get_int(0x2007, 0x00))
        return float(cmx) / float(cdv)

    def _cmd_pulses_per_rev(self) -> float:
        """Command pulses per motor revolution = EncoderResolution / (2006h / 2007h)."""
        return max(1.0, self._encoder_ppr() / self._gear_ratio())

    def _rpm001_to_pps(self, vel_001rpm: float) -> float:
        rpm = vel_001rpm * 0.01
        return (rpm / 60.0) * self._cmd_pulses_per_rev()

    def _pps_to_rpm001(self, vel_pps: float) -> float:
        cmd_ppr = self._cmd_pulses_per_rev()
        rpm = (vel_pps / cmd_ppr) * 60.0
        return rpm * 100.0

    def _vel_od_to_pps(self, od_val: float) -> float:
        pa54 = self.od.get_int(0x2036, 0x00)
        if pa54 == 0:
            return self._rpm001_to_pps(od_val)
        return float(od_val)

    def _pps_to_vel_od(self, vel_pps: float) -> int:
        pa54 = self.od.get_int(0x2036, 0x00)
        if pa54 == 0:
            return int(round(self._pps_to_rpm001(vel_pps)))
        return int(round(vel_pps))

    def _accel_od_to_pps2(self, od_val: float) -> float:
        val = max(1.0, float(od_val))
        pa54 = self.od.get_int(0x2036, 0x00)
        if pa54 == 0:
            rated_pps = (RATED_SPEED_RPM / 60.0) * self._cmd_pulses_per_rev()
            return rated_pps / (val / 1000.0)
        return val

    def _velocity_limit_pps(self, apply_2d20: bool = False) -> float:
        v_6080_001rpm = max(1, self.od.get_int(0x6080, 0x00))
        if apply_2d20:
            # 2D20h (Velocity limit value) is active ONLY in Pt (Profile Torque) mode
            v_2d20_001rpm = max(0, self.od.get_int(0x2D20, 0x00))
            limit_001rpm = min(v_2d20_001rpm, v_6080_001rpm)
        else:
            limit_001rpm = v_6080_001rpm
        limit_pps = self._rpm001_to_pps(limit_001rpm)

        v_607f = self._vel_od_to_pps(self.od.get_int(0x607F, 0x00))
        if v_607f > 0:
            limit_pps = min(limit_pps, v_607f)
        return max(0.0 if apply_2d20 else 100.0, limit_pps)

    def process_cyclic_input(self, ry_bytes: bytes, rww_bytes: bytes) -> Tuple[bytes, bytes]:
        """處理單次 61450 循環封包輸入 (RY/RWw)，更新 CiA 402 狀態機並回傳 (RX/RWr)。"""
        if len(ry_bytes) < 8:
            ry_bytes = ry_bytes.ljust(8, b"\x00")
        if len(rww_bytes) < 64:
            rww_bytes = rww_bytes.ljust(64, b"\x00")

        self.last_ry_bytes = ry_bytes
        self.last_rww_bytes = rww_bytes
        self.cyclic_rx_count += 1
        self.last_cyclic_time = time.time()

        # Check RY(n+3)F (Bit 63 = MSB of byte 7): CSR
        csr = bool((ry_bytes[7] >> 7) & 0x01)
        if csr != self.csr_active:
            self.csr_active = csr
            self.od.set_pdo_locked(csr)
            if self.on_event:
                self.on_event(self.axis_id, f"CSR (RY(n+3)F) -> {int(csr)}, SSR -> {int(csr)}")

        if self.csr_active:
            self.od.unpack_rx_pdo(rww_bytes)
            self.ssr_active = True
            prev_mode = self._prev_op_mode
            req_mode = self.od.get_int(0x6060, 0x00)
            if req_mode in (OpMode.NO_MODE, OpMode.PP, OpMode.PV, OpMode.PT, OpMode.HM):
                self.od.set_internal(0x6061, 0x00, req_mode)
            active_mode = self.od.get_int(0x6061, 0x00)
            if active_mode != prev_mode:
                self._on_mode_changed(prev_mode, active_mode)
                self._prev_op_mode = active_mode

            prev_cw_before_pds = self._prev_pds_cw
            cw_now = self.od.get_int(0x6040, 0x00)
            self._step_pds_state_machine(cw_now)
            if self.pds_state == PDSState.OPERATION_ENABLED:
                if active_mode == OpMode.PP:
                    if bool(cw_now & 0x0010) and (
                        not bool(prev_cw_before_pds & 0x0010) or prev_mode != OpMode.PP
                    ):
                        self._trigger_pp_move(cw_now)
                        self._pp_sp_latched_in_cyclic = True
                    elif not bool(cw_now & 0x0010):
                        self._pp_sp_ack = False
                elif active_mode == OpMode.HM:
                    if bool(cw_now & 0x0010) and (
                        not bool(prev_cw_before_pds & 0x0010) or prev_mode != OpMode.HM
                    ):
                        self._trigger_hm_start()
                        self._hm_start_latched_in_cyclic = True
                elif active_mode == OpMode.PV:
                    self._sync_pv_target_reached(cw_now)
        else:
            self.ssr_active = False

        self._update_status_objects()
        rx_bytes = self._build_rx_bytes()
        rwr_bytes = self.od.pack_tx_pdo()
        self.last_rx_bytes = rx_bytes
        self.last_rwr_bytes = rwr_bytes
        self.cyclic_tx_count += 1
        return rx_bytes, rwr_bytes

    def _build_rx_bytes(self) -> bytes:
        """
        組裝 Slave -> Master 64-bit RX 位元資料（對齊實機 SDC-F 監看結果）：
        - 通訊就緒且無異警：RX3F (SSR) + RX3E (Remote Ready) = 1 -> rx[7] = 0xC0 (GX Works3 B30 = -16384)
        - 發生伺服異警：額外拉高 RX3B -> rx[7] = 0xC8
        - 其餘 RX00~RX3D 位元保持為 0
        """
        rx = bytearray(8)
        if self.pds_state in (PDSState.FAULT, PDSState.FAULT_REACTION_ACTIVE):
            rx[7] |= 0x08  # RX(n+3)B：伺服異警旗標
        if self.ssr_active:
            rx[7] |= 0xC0  # RX(n+3)F (SSR) + RX(n+3)E (從站初始/遠端就緒旗標，實機 B30=0xC000)
        return bytes(rx)

    def inject_fault(self, error_code: int, fault_name: str = "") -> None:
        self._fault_code = error_code & 0xFFFF
        default_name = (
            f"AL.{self._fault_code & 0xFF:02X}"
            if (self._fault_code & 0xFF00) == 0xFF00
            else f"AL.{self._fault_code:04X}"
        )
        self._fault_name = fault_name or default_name
        self.od.set_internal(0x603F, 0x00, self._fault_code)
        self.od.set_internal(0x1001, 0x00, 0x01)
        self.pds_state = PDSState.FAULT
        self._pp_active = False
        self._hm_phase = HomingPhase.IDLE
        self._vel_demand_pps = 0.0
        self._vel_actual_pps = 0.0
        self._torque_actual_01pct = 0.0
        self._torque_limiting = False
        self._vel_limiting = False
        self._update_status_objects()
        if self.on_event:
            self.on_event(self.axis_id, f"Fault Injected: {self._fault_name} (603Fh=0x{self._fault_code:04X})")

    def clear_fault(self) -> None:
        self._fault_code = 0x0000
        self._fault_name = ""
        self.od.set_internal(0x603F, 0x00, 0x0000)
        self.od.set_internal(0x1001, 0x00, 0x00)
        if self.pds_state in (PDSState.FAULT, PDSState.FAULT_REACTION_ACTIVE):
            self.pds_state = PDSState.SWITCH_ON_DISABLED
        self._update_status_objects()

    def _reset_on_link_down(self, reason: str) -> None:
        self.csr_active = False
        self.ssr_active = False
        self.od.set_pdo_locked(False)
        if self.pds_state not in (PDSState.FAULT, PDSState.FAULT_REACTION_ACTIVE):
            self.pds_state = PDSState.SWITCH_ON_DISABLED
        self._prev_cw = 0
        self._prev_pds_cw = 0
        self._prev_op_mode = OpMode.NO_MODE
        self.od.set_internal(0x6040, 0x00, 0x0000)
        self.od.set_internal(0x6060, 0x00, OpMode.NO_MODE)
        self.od.set_internal(0x6061, 0x00, OpMode.NO_MODE)
        self._pp_active = False
        self._pp_sp_ack = False
        self._pp_sp_latched_in_cyclic = False
        self._pp_target_reached = True
        self._hm_phase = HomingPhase.IDLE
        self._hm_start_latched_in_cyclic = False
        self._hm_waiting_orgp_fall = False
        self._hm_need_leave_orgp = False
        self._hm_saw_orgp_wrong_side = False
        self._hm_slow_approach = False
        self._vel_demand_pps = 0.0
        self._vel_actual_pps = 0.0
        self._torque_actual_01pct = 0.0
        self._torque_limiting = False
        self._vel_limiting = False
        self._pos_demand_f = self._pos_actual_f
        self.last_ry_bytes = b"\x00" * 8
        self.last_rww_bytes = b"\x00" * 64
        self._update_status_objects()
        self.last_rx_bytes = self._build_rx_bytes()
        self.last_rwr_bytes = self.od.pack_tx_pdo()
        if self.on_event:
            self.on_event(self.axis_id, reason)

    def on_communication_lost(self, reason: str = "Cyclic Watchdog Timeout") -> None:
        self._reset_on_link_down(f"通訊中斷重置 ({reason}) -> CSR=0, SSR=0, Switch On Disabled")

    def on_communication_stopped(self) -> None:
        self._reset_on_link_down("停止通訊重置 -> CSR=0, SSR=0, Switch On Disabled")

    def set_sensor(self, name: str, active: bool) -> None:
        key = name.lower()
        if key == "lsp":
            self.sensors.lsp = active
        elif key == "lsn":
            self.sensors.lsn = active
        elif key == "orgp":
            self.sensors.orgp = active
        elif key == "z_phase":
            self.sensors.z_phase = active
        self._update_status_objects()

    def step(self, dt: float) -> None:
        """伺服物理引擎單步積分（預設 10ms 週期），更新 PDS、各運轉模式軌跡與追隨誤差。"""
        dt = max(0.001, min(0.05, dt))
        cw = self.od.get_int(0x6040, 0x00)
        self._step_pds_state_machine(cw)

        prev_mode_step = self._prev_op_mode
        req_mode = self.od.get_int(0x6060, 0x00)
        if req_mode in (OpMode.NO_MODE, OpMode.PP, OpMode.PV, OpMode.PT, OpMode.HM):
            self.od.set_internal(0x6061, 0x00, req_mode)

        active_mode = self.od.get_int(0x6061, 0x00)
        entered_mode = active_mode != prev_mode_step
        if entered_mode:
            self._on_mode_changed(prev_mode_step, active_mode)
            self._prev_op_mode = active_mode

        if self.pds_state == PDSState.OPERATION_ENABLED:
            if active_mode == OpMode.PP:
                self._step_pp_mode(cw, dt)
            elif active_mode == OpMode.HM:
                self._step_hm_mode(cw, dt, entered_hm=(entered_mode and active_mode == OpMode.HM))
            elif active_mode == OpMode.PV:
                self._step_pv_mode(cw, dt)
            elif active_mode == OpMode.PT:
                self._step_pt_mode(cw, dt)
            else:
                self._coast_to_stop(dt)
        elif self.pds_state == PDSState.QUICK_STOP_ACTIVE:
            self._step_quick_stop(dt)
        else:
            self._coast_to_stop(dt)

        if self.pds_state == PDSState.OPERATION_ENABLED:
            if active_mode == OpMode.PT:
                self._pos_demand_f = self._pos_actual_f
            else:
                tau = 0.012
                alpha = 1.0 - math.exp(-dt / tau)
                self._pos_actual_f += (self._pos_demand_f - self._pos_actual_f) * alpha
                if abs(self._vel_demand_pps) < 1.0 and abs(self._pos_demand_f - self._pos_actual_f) < 0.8:
                    self._pos_actual_f = self._pos_demand_f
        else:
            self._pos_demand_f = self._pos_actual_f

        cmd_ppr = self._cmd_pulses_per_rev()
        self._hm_prev_rev = int(math.floor(self._pos_actual_f / cmd_ppr))

        self._motor_angle_deg = ((self._pos_actual_f % cmd_ppr) / cmd_ppr) * 360.0

        following_err = int(round(self._pos_demand_f - self._pos_actual_f))
        pc05_limit = self.od.get_int(0x2205, 0x00)
        if self.pds_state == PDSState.OPERATION_ENABLED and abs(following_err) > pc05_limit:
            self.inject_fault(0xFF08, "AL.08 Excessive Following Error")

        self._prev_cw = cw
        self._update_status_objects()

    def _step_pds_state_machine(self, cw: int) -> None:
        prev_state = self.pds_state
        fault_reset_edge = bool(cw & 0x0080) and not bool(self._prev_pds_cw & 0x0080)
        enable_voltage = bool(cw & 0x0002)
        quick_stop_n = bool(cw & 0x0004)
        switch_on = bool(cw & 0x0001)
        enable_op = bool(cw & 0x0008)

        if self.pds_state == PDSState.FAULT:
            if fault_reset_edge:
                self.clear_fault()
        elif self.pds_state == PDSState.NOT_READY_TO_SWITCH_ON:
            self.pds_state = PDSState.SWITCH_ON_DISABLED
        elif self.pds_state == PDSState.SWITCH_ON_DISABLED:
            if enable_voltage and quick_stop_n:
                if not switch_on:
                    self.pds_state = PDSState.READY_TO_SWITCH_ON
                elif not (self._prev_pds_cw & 0x0001):
                    if enable_op:
                        self.pds_state = PDSState.OPERATION_ENABLED
                        self._on_enter_operation_enabled()
                    else:
                        self.pds_state = PDSState.SWITCHED_ON
        elif self.pds_state == PDSState.READY_TO_SWITCH_ON:
            if not enable_voltage or not quick_stop_n:
                self.pds_state = PDSState.SWITCH_ON_DISABLED
            elif switch_on:
                if enable_op:
                    self.pds_state = PDSState.OPERATION_ENABLED
                    self._on_enter_operation_enabled()
                else:
                    self.pds_state = PDSState.SWITCHED_ON
        elif self.pds_state == PDSState.SWITCHED_ON:
            if not enable_voltage or not quick_stop_n:
                self.pds_state = PDSState.SWITCH_ON_DISABLED
            elif not switch_on:
                self.pds_state = PDSState.READY_TO_SWITCH_ON
            elif enable_op:
                self.pds_state = PDSState.OPERATION_ENABLED
                self._on_enter_operation_enabled()
        elif self.pds_state == PDSState.OPERATION_ENABLED:
            if not enable_voltage:
                self.pds_state = PDSState.SWITCH_ON_DISABLED
            elif not quick_stop_n:
                self.pds_state = PDSState.QUICK_STOP_ACTIVE
            elif not switch_on:
                self.pds_state = PDSState.READY_TO_SWITCH_ON
            elif not enable_op:
                self.pds_state = PDSState.SWITCHED_ON
        elif self.pds_state == PDSState.QUICK_STOP_ACTIVE:
            if not enable_voltage:
                self.pds_state = PDSState.SWITCH_ON_DISABLED
            elif abs(self._vel_actual_pps) < 5.0:
                self._vel_actual_pps = 0.0
                self._vel_demand_pps = 0.0
                self.pds_state = PDSState.SWITCH_ON_DISABLED

        self._prev_pds_cw = cw

        if self.pds_state != prev_state and self.on_event:
            self.on_event(
                self.axis_id,
                f"PDS: {prev_state.label} -> {self.pds_state.label} "
                f"(6040h=0x{cw:04X}, 6060h={self.od.get_int(0x6060, 0x00)})",
            )

    def _on_enter_operation_enabled(self) -> None:
        self._pos_demand_f = self._pos_actual_f
        self._pp_target_pos = int(round(self._pos_actual_f))
        self._pp_active = False
        self._pp_sp_ack = False
        self._pp_sp_latched_in_cyclic = False
        self._pp_target_reached = True
        self._hm_phase = HomingPhase.IDLE
        self._hm_start_latched_in_cyclic = False
        self._hm_waiting_orgp_fall = False
        self._hm_need_leave_orgp = False
        self._hm_saw_orgp_wrong_side = False
        self._hm_slow_approach = False

    def _on_mode_changed(self, prev_mode: int, new_mode: int) -> None:
        if prev_mode == new_mode:
            return
        if prev_mode == OpMode.PP:
            self._pp_active = False
            self._pp_sp_ack = False
            self._pp_sp_latched_in_cyclic = False
        elif prev_mode == OpMode.HM:
            self._hm_start_latched_in_cyclic = False
            self._hm_waiting_orgp_fall = False
            self._hm_need_leave_orgp = False
            self._hm_saw_orgp_wrong_side = False
            self._hm_slow_approach = False
            if self._hm_phase in (HomingPhase.SEARCH_SWITCH, HomingPhase.SEARCH_ZERO):
                self._hm_phase = HomingPhase.IDLE

        if new_mode == OpMode.HM:
            self._hm_phase = HomingPhase.IDLE
            self._hm_attained = False
            self._hm_error = False
            self._hm_switch_hit = False
            self._hm_start_latched_in_cyclic = False
            self._hm_waiting_orgp_fall = False
            self._hm_need_leave_orgp = False
            self._hm_saw_orgp_wrong_side = False
            self._hm_slow_approach = False
            self._pp_active = False
            self._pp_sp_ack = False
            self._pp_sp_latched_in_cyclic = False
            self._pp_target_reached = True
        elif new_mode == OpMode.PV:
            self._pp_active = False
            self._pp_sp_ack = False
            self._pp_sp_latched_in_cyclic = False
            if self.pds_state == PDSState.OPERATION_ENABLED:
                self._sync_pv_target_reached(self.od.get_int(0x6040, 0x00))
            else:
                self._pp_target_reached = True
        elif new_mode == OpMode.PP:
            self._pp_active = False
            self._pp_sp_ack = False
            self._pp_target_reached = True
        elif new_mode == OpMode.PT:
            self._pp_active = False
            self._pp_sp_ack = False
            cmd_torque = float(self.od.get_int(0x6071, 0x00))
            self._pp_target_reached = abs(self._torque_actual_01pct - cmd_torque) < 5.0
        elif new_mode == OpMode.NO_MODE:
            self._pp_active = False
            self._pp_sp_ack = False
            self._pp_sp_latched_in_cyclic = False
            self._hm_start_latched_in_cyclic = False
            self._pp_target_reached = True
            self._vel_demand_pps = 0.0
        if new_mode != OpMode.PT:
            self._torque_limiting = False
            self._vel_limiting = False

    def _pv_window_pps(self) -> float:
        win_od = max(0, self.od.get_int(0x606D, 0x00))
        if win_od == 0:
            return 50.0
        return max(20.0, self._vel_od_to_pps(win_od))

    def _zero_speed_pps(self) -> float:
        thr_od = max(0, self.od.get_int(0x606F, 0x00))
        if thr_od == 0:
            return 10.0
        return max(10.0, self._vel_od_to_pps(thr_od))

    def _pv_target_pps(self, cw: int) -> Tuple[float, float]:
        halt = bool(cw & 0x0100)
        target_od = 0 if halt else self.od.get_int(0x60FF, 0x00)
        target_pps = self._vel_od_to_pps(target_od)
        limit_pps = self._velocity_limit_pps(apply_2d20=False)
        target_pps = max(-limit_pps, min(limit_pps, target_pps))
        return target_pps, limit_pps

    def _sync_pv_target_reached(self, cw: int) -> None:
        target_pps, _ = self._pv_target_pps(cw)
        tol = max(self._pv_window_pps(), abs(target_pps) * 0.02)
        self._pp_target_reached = abs(self._vel_actual_pps - target_pps) <= tol

    def _effective_sensors(self) -> Tuple[bool, bool, bool]:
        pos = self._pos_actual_f
        v_en = self.sensors.virtual_sensors_enabled
        lsn = self.sensors.lsn or (v_en and pos <= self.sensors.virtual_lsn_pos)
        lsp = self.sensors.lsp or (v_en and pos >= self.sensors.virtual_lsp_pos)
        orgp = self.sensors.orgp or (
            v_en and (self.sensors.virtual_orgp_min <= pos <= self.sensors.virtual_orgp_max)
        )
        return lsn, lsp, orgp

    def _clamp_limits(self, prev_pos: float, next_pos: float, vel_pps: float) -> Tuple[float, float, bool]:
        """檢查正負極限 (LSP/LSN) 與軟體極限 (607Dh)；受阻方向凍結於原位置且速度歸零。"""
        lsn, lsp, _ = self._effective_sensors()
        sw_min = float(self.od.get_int(0x607D, 0x01))
        sw_max = float(self.od.get_int(0x607D, 0x02))
        clamped = False

        if (lsp or next_pos >= sw_max) and vel_pps > 0:
            next_pos = min(prev_pos, sw_max)
            vel_pps = 0.0
            clamped = True
        if (lsn or next_pos <= sw_min) and vel_pps < 0:
            next_pos = max(prev_pos, sw_min)
            vel_pps = 0.0
            clamped = True
        return next_pos, vel_pps, clamped

    def _trigger_pp_move(self, cw: int) -> None:
        is_relative = bool(cw & 0x0040)
        cmd_pos = self.od.get_int(0x607A, 0x00)
        if is_relative:
            self._pp_target_pos = int(round(self._pos_demand_f)) + cmd_pos
        else:
            self._pp_target_pos = cmd_pos
        self._pp_active = True
        self._pp_sp_ack = True
        self._pp_target_reached = False
        self._pp_in_pos_timer = 0.0

    def _step_pp_mode(self, cw: int, dt: float) -> None:
        new_sp = bool(cw & 0x0010)
        prev_sp = bool(self._prev_cw & 0x0010)
        halt = bool(cw & 0x0100)

        if self._pp_sp_latched_in_cyclic:
            self._pp_sp_latched_in_cyclic = False
            if not new_sp:
                self._pp_sp_ack = False
        elif new_sp and not prev_sp:
            self._trigger_pp_move(cw)
        elif not new_sp:
            self._pp_sp_ack = False

        if halt:
            decel = self._accel_od_to_pps2(self.od.get_int(0x6084, 0x00))
            prev_pos = self._pos_demand_f
            self._vel_demand_pps = self._approach_zero(self._vel_demand_pps, decel, dt)
            next_pos = prev_pos + self._vel_demand_pps * dt
            next_pos, self._vel_demand_pps, _ = self._clamp_limits(prev_pos, next_pos, self._vel_demand_pps)
            self._pos_demand_f = next_pos
            self._vel_actual_pps = self._vel_demand_pps
            return

        max_vel = min(
            abs(self._vel_od_to_pps(self.od.get_int(0x6081, 0x00))),
            self._velocity_limit_pps(apply_2d20=False),
        )
        accel = self._accel_od_to_pps2(self.od.get_int(0x6083, 0x00))
        decel = self._accel_od_to_pps2(self.od.get_int(0x6084, 0x00))

        dist = float(self._pp_target_pos) - self._pos_demand_f
        abs_dist = abs(dist)
        lsn, lsp, _ = self._effective_sensors()
        sw_min = float(self.od.get_int(0x607D, 0x01))
        sw_max = float(self.od.get_int(0x607D, 0x02))

        if abs_dist <= 0.5 and abs(self._vel_demand_pps) <= accel * dt * 1.5:
            self._pos_demand_f = float(self._pp_target_pos)
            self._vel_demand_pps = 0.0
            self._vel_actual_pps = 0.0
            self._pp_active = False
        else:
            direction = 1.0 if dist > 0 else -1.0
            v_stop = math.sqrt(max(0.0, 2.0 * decel * abs_dist))
            desired_v = direction * min(max_vel, v_stop)
            blocked_pos = (lsp or self._pos_demand_f >= sw_max) and (desired_v > 0 or self._vel_demand_pps > 0)
            blocked_neg = (lsn or self._pos_demand_f <= sw_min) and (desired_v < 0 or self._vel_demand_pps < 0)

            if blocked_pos or blocked_neg:
                self._vel_demand_pps = 0.0
                self._vel_actual_pps = 0.0
                self._pos_actual_f = min(max(self._pos_actual_f, sw_min), sw_max)
                self._pos_demand_f = self._pos_actual_f
            else:
                if direction * self._vel_demand_pps < 0:
                    self._vel_demand_pps = self._approach_zero(self._vel_demand_pps, decel, dt)
                elif abs(desired_v) > abs(self._vel_demand_pps):
                    step_v = accel * dt
                    if abs(desired_v - self._vel_demand_pps) <= step_v:
                        self._vel_demand_pps = desired_v
                    else:
                        self._vel_demand_pps += direction * step_v
                else:
                    step_v = decel * dt
                    if abs(desired_v - self._vel_demand_pps) <= step_v:
                        self._vel_demand_pps = desired_v
                    else:
                        self._vel_demand_pps -= math.copysign(step_v, self._vel_demand_pps)

                prev_pos = self._pos_demand_f
                next_pos = prev_pos + self._vel_demand_pps * dt
                if (direction > 0 and next_pos >= self._pp_target_pos) or (
                    direction < 0 and next_pos <= self._pp_target_pos
                ):
                    next_pos = float(self._pp_target_pos)
                    self._vel_demand_pps = 0.0

                next_pos, self._vel_demand_pps, clamped = self._clamp_limits(prev_pos, next_pos, self._vel_demand_pps)
                if clamped and (lsp or lsn):
                    self._pos_actual_f = min(max(self._pos_actual_f, sw_min), sw_max)
                    next_pos = self._pos_actual_f
                self._pos_demand_f = next_pos
                self._vel_actual_pps = self._vel_demand_pps

        pos_window = self.od.get_int(0x6067, 0x00)
        pos_win_time_s = self.od.get_int(0x6068, 0x00) / 1000.0
        if not self._pp_active and abs(self._pp_target_pos - self._pos_actual_f) <= pos_window:
            self._pp_in_pos_timer += dt
            if self._pp_in_pos_timer >= pos_win_time_s:
                self._pp_target_reached = True
        else:
            self._pp_in_pos_timer = 0.0
            if self._pp_active:
                self._pp_target_reached = False

        self._torque_actual_01pct = self._estimate_torque_from_vel(self._vel_actual_pps, max_vel)

    @staticmethod
    def _hm_orgp_spec(method: int) -> Tuple[int, int, str, bool, bool]:
        """
        回傳 CiA 402 原點開關 (ORGP) 系列方法 (3..14, 19..30) 之規格：
        (init_dir, target_dir, target_edge, allow_lsn_rev, allow_lsp_rev)
        - init_dir:      當未觸及 ORGP 時的初始搜尋方向 (+1 / -1)
        - target_dir:    偵測目標 ORGP 邊緣 (及後續尋 Z) 之移動方向 (+1 / -1)
        - target_edge:   'rising' (0->1) 或 'falling' (1->0)
        - allow_lsn_rev: 碰到負極限 LSN 時是否反轉為 +1 (False 表示報錯停止)
        - allow_lsp_rev: 碰到正極限 LSP 時是否反轉為 -1 (False 表示報錯停止)
        """
        base = method - 16 if 19 <= method <= 30 else method
        specs: Dict[int, Tuple[int, int, str, bool, bool]] = {
            3: (1, -1, "falling", False, False),
            4: (1, 1, "rising", False, False),
            5: (-1, 1, "falling", False, False),
            6: (-1, -1, "rising", False, False),
            7: (1, -1, "falling", False, True),
            8: (1, 1, "rising", False, True),
            9: (1, -1, "rising", False, True),
            10: (1, 1, "falling", False, True),
            11: (-1, 1, "falling", True, False),
            12: (-1, -1, "rising", True, False),
            13: (-1, 1, "rising", True, False),
            14: (-1, -1, "falling", True, False),
        }
        return specs[base]

    def _enter_hm_error(self, msg: str) -> None:
        """原點復歸觸及不允許之極限或無效方式時停止移動並設定 Statusword Bit 13。"""
        self._vel_demand_pps = 0.0
        self._vel_actual_pps = 0.0
        self._pos_demand_f = self._pos_actual_f
        self._hm_phase = HomingPhase.ERROR
        self._hm_error = True
        self._hm_waiting_orgp_fall = False
        self._hm_need_leave_orgp = False
        self._hm_saw_orgp_wrong_side = False
        self._hm_slow_approach = False
        self._pp_target_reached = True
        if self.on_event:
            self.on_event(self.axis_id, msg)

    def _trigger_hm_start(self) -> None:
        method = self.od.get_int(0x6098, 0x00)
        offset = self.od.get_int(0x607C, 0x00)
        lsn, lsp, orgp = self._effective_sensors()
        self._hm_attained = False
        self._hm_error = False
        self._hm_switch_hit = False
        self._hm_waiting_orgp_fall = False
        self._hm_need_leave_orgp = False
        self._hm_saw_orgp_wrong_side = False
        self._hm_slow_approach = False
        self._hm_prev_lsn = lsn
        self._hm_prev_lsp = lsp
        self._hm_prev_orgp = orgp

        if method in (35, 37):
            self._finish_homing(f"Homing Method {method} Complete (Offset={offset})")
        elif method in (33, 34):
            self._hm_search_dir = -1 if method == 33 else 1
            if (method == 33 and lsn) or (method == 34 and lsp):
                lim_name = "LSN" if method == 33 else "LSP"
                self._enter_hm_error(f"Homing Method {method} Error: {lim_name} active at start")
                return
            self._pp_target_reached = False
            self._hm_phase = HomingPhase.SEARCH_ZERO
            if self.on_event:
                self.on_event(self.axis_id, f"Homing Method {method} Started (dir={self._hm_search_dir:+d})")
        elif method in (1, 17):
            self._pp_target_reached = False
            if lsn:
                # 起始已壓在 LSN 上：直接進入慢速正向 (+1) 脫離 LSN 階段
                self._hm_search_dir = 1
                self._hm_phase = HomingPhase.SEARCH_ZERO
            else:
                self._hm_search_dir = -1
                self._hm_phase = HomingPhase.SEARCH_SWITCH
            if self.on_event:
                self.on_event(self.axis_id, f"Homing Method {method} Started (dir={self._hm_search_dir:+d})")
        elif method in (2, 18):
            self._pp_target_reached = False
            if lsp:
                # 起始已壓在 LSP 上：直接進入慢速負向 (-1) 脫離 LSP 階段
                self._hm_search_dir = -1
                self._hm_phase = HomingPhase.SEARCH_ZERO
            else:
                self._hm_search_dir = 1
                self._hm_phase = HomingPhase.SEARCH_SWITCH
            if self.on_event:
                self.on_event(self.axis_id, f"Homing Method {method} Started (dir={self._hm_search_dir:+d})")
        elif (3 <= method <= 14) or (19 <= method <= 30):
            init_dir, target_dir, target_edge, allow_lsn_rev, allow_lsp_rev = self._hm_orgp_spec(method)
            self._pp_target_reached = False
            if orgp:
                if target_edge == "falling":
                    self._hm_waiting_orgp_fall = True
                    self._hm_search_dir = target_dir
                    self._hm_slow_approach = True
                else:
                    self._hm_need_leave_orgp = True
                    self._hm_search_dir = -target_dir
                    self._hm_slow_approach = True
            else:
                self._hm_search_dir = init_dir
                if self._hm_search_dir < 0 and lsn:
                    if allow_lsn_rev:
                        self._hm_search_dir = 1
                    else:
                        self._enter_hm_error(f"Homing Method {method} Error: LSN active at start")
                        return
                elif self._hm_search_dir > 0 and lsp:
                    if allow_lsp_rev:
                        self._hm_search_dir = -1
                    else:
                        self._enter_hm_error(f"Homing Method {method} Error: LSP active at start")
                        return
            self._hm_phase = HomingPhase.SEARCH_SWITCH
            if self.on_event:
                self.on_event(self.axis_id, f"Homing Method {method} Started (dir={self._hm_search_dir:+d})")
        else:
            self._enter_hm_error(f"Homing Method {method} Error: Unsupported method")

    def _step_hm_mode(self, cw: int, dt: float, entered_hm: bool = False) -> None:
        start_homing = bool(cw & 0x0010)
        prev_start = bool(self._prev_cw & 0x0010)
        halt = bool(cw & 0x0100)

        lsn, lsp, orgp = self._effective_sensors()
        lsn_rising = lsn and not self._hm_prev_lsn
        lsp_rising = lsp and not self._hm_prev_lsp
        orgp_rising = orgp and not self._hm_prev_orgp

        if self._hm_start_latched_in_cyclic:
            self._hm_start_latched_in_cyclic = False
        elif start_homing and (not prev_start or entered_hm):
            self._trigger_hm_start()
            lsn, lsp, orgp = self._effective_sensors()
            lsn_rising = False
            lsp_rising = False
            orgp_rising = False

        if not start_homing and self._hm_phase not in (HomingPhase.IDLE, HomingPhase.DONE, HomingPhase.ERROR):
            self._hm_phase = HomingPhase.IDLE
            self._hm_waiting_orgp_fall = False
            self._hm_need_leave_orgp = False
            self._hm_saw_orgp_wrong_side = False
            self._hm_slow_approach = False
            self._vel_demand_pps = 0.0
            self._vel_actual_pps = 0.0
            self._pos_demand_f = self._pos_actual_f
            self._pp_target_reached = True

        if halt and self._hm_phase in (HomingPhase.SEARCH_SWITCH, HomingPhase.SEARCH_ZERO):
            decel = self._accel_od_to_pps2(self.od.get_int(0x609A, 0x00))
            self._vel_demand_pps = self._approach_zero(self._vel_demand_pps, decel, dt)
            next_pos = self._pos_demand_f + self._vel_demand_pps * dt
            next_pos, self._vel_demand_pps, _ = self._clamp_limits(self._pos_demand_f, next_pos, self._vel_demand_pps)
            self._pos_demand_f = next_pos
            self._vel_actual_pps = self._vel_demand_pps
            self._pp_target_reached = abs(self._vel_actual_pps) < 1.0
        elif self._hm_phase == HomingPhase.SEARCH_SWITCH:
            self._pp_target_reached = False
            method = self.od.get_int(0x6098, 0x00)
            v_switch = min(abs(self._vel_od_to_pps(self.od.get_int(0x6099, 0x01))), self._velocity_limit_pps(apply_2d20=False))
            v_zero = min(abs(self._vel_od_to_pps(self.od.get_int(0x6099, 0x02))), self._velocity_limit_pps(apply_2d20=False))
            accel = self._accel_od_to_pps2(self.od.get_int(0x609A, 0x00))
            skip_motion = False

            if method in (1, 17):
                if lsn or lsn_rising:
                    self._hm_search_dir = 1
                    self._hm_phase = HomingPhase.SEARCH_ZERO
                    self._vel_demand_pps = 0.0
                    self._vel_actual_pps = 0.0
                    self._pos_demand_f = self._pos_actual_f
                    skip_motion = True
                elif lsp and self._hm_search_dir > 0:
                    self._enter_hm_error(f"Homing Method {method} Error: Hit LSP unexpectedly")
                    skip_motion = True
            elif method in (2, 18):
                if lsp or lsp_rising:
                    self._hm_search_dir = -1
                    self._hm_phase = HomingPhase.SEARCH_ZERO
                    self._vel_demand_pps = 0.0
                    self._vel_actual_pps = 0.0
                    self._pos_demand_f = self._pos_actual_f
                    skip_motion = True
                elif lsn and self._hm_search_dir < 0:
                    self._enter_hm_error(f"Homing Method {method} Error: Hit LSN unexpectedly")
                    skip_motion = True
            elif (3 <= method <= 14) or (19 <= method <= 30):
                init_dir, target_dir, target_edge, allow_lsn_rev, allow_lsp_rev = self._hm_orgp_spec(method)
                no_z = 19 <= method <= 30

                # 1) 極限檢查 (LSN / LSP)
                if lsn and self._hm_search_dir < 0:
                    if allow_lsn_rev and not (self._hm_waiting_orgp_fall or self._hm_need_leave_orgp):
                        self._hm_search_dir = 1
                        self._hm_saw_orgp_wrong_side = False
                        self._vel_demand_pps = 0.0
                        self._vel_actual_pps = 0.0
                        self._pos_demand_f = self._pos_actual_f
                        skip_motion = True
                    else:
                        self._enter_hm_error(f"Homing Method {method} Error: Hit LSN (no ORGP)")
                        skip_motion = True
                elif lsp and self._hm_search_dir > 0:
                    if allow_lsp_rev and not (self._hm_waiting_orgp_fall or self._hm_need_leave_orgp):
                        self._hm_search_dir = -1
                        self._hm_saw_orgp_wrong_side = False
                        self._vel_demand_pps = 0.0
                        self._vel_actual_pps = 0.0
                        self._pos_demand_f = self._pos_actual_f
                        skip_motion = True
                    else:
                        self._enter_hm_error(f"Homing Method {method} Error: Hit LSP (no ORGP)")
                        skip_motion = True

                # 2) ORGP 邊緣狀態機檢查
                if not skip_motion:
                    if target_edge == "rising":
                        if self._hm_need_leave_orgp:
                            self._hm_search_dir = -target_dir
                            self._hm_slow_approach = True
                            if not orgp:
                                self._hm_need_leave_orgp = False
                                self._hm_search_dir = target_dir
                                self._hm_slow_approach = True
                                self._vel_demand_pps = 0.0
                                self._vel_actual_pps = 0.0
                                self._pos_demand_f = self._pos_actual_f
                                skip_motion = True
                        elif self._hm_search_dir == target_dir:
                            if orgp_rising or orgp:
                                self._hm_switch_hit = True
                                self._hm_slow_approach = False
                                if no_z:
                                    self._finish_homing()
                                    skip_motion = True
                                else:
                                    self._hm_phase = HomingPhase.SEARCH_ZERO
                                    self._hm_search_dir = target_dir
                        else:
                            # 從反方向穿越 ORGP 後回頭捕捉上升緣
                            if orgp:
                                self._hm_saw_orgp_wrong_side = True
                            elif self._hm_saw_orgp_wrong_side:
                                self._hm_saw_orgp_wrong_side = False
                                self._hm_search_dir = target_dir
                                self._hm_slow_approach = True
                                self._vel_demand_pps = 0.0
                                self._vel_actual_pps = 0.0
                                self._pos_demand_f = self._pos_actual_f
                                skip_motion = True
                    else:
                        # target_edge == "falling"
                        if self._hm_waiting_orgp_fall:
                            self._hm_search_dir = target_dir
                            self._hm_slow_approach = True
                            if not orgp:
                                self._hm_waiting_orgp_fall = False
                                self._hm_switch_hit = True
                                self._hm_slow_approach = False
                                if no_z:
                                    self._finish_homing()
                                    skip_motion = True
                                else:
                                    self._hm_phase = HomingPhase.SEARCH_ZERO
                                    self._hm_search_dir = target_dir
                        elif self._hm_search_dir == target_dir:
                            if orgp:
                                self._hm_waiting_orgp_fall = True
                                self._hm_slow_approach = True
                        else:
                            if init_dir != target_dir:
                                # 如 Method 3/5/7/11 (19/21/23/27)：正(負)向碰 ORGP 立即反轉慢速脫離
                                if orgp or orgp_rising:
                                    self._hm_search_dir = target_dir
                                    self._hm_waiting_orgp_fall = True
                                    self._hm_slow_approach = True
                                    self._vel_demand_pps = 0.0
                                    self._vel_actual_pps = 0.0
                                    self._pos_demand_f = self._pos_actual_f
                                    skip_motion = True
                            else:
                                # 如 Method 10/14 (26/30) 反轉折返後需先穿過 ORGP 再順向進出
                                if orgp:
                                    self._hm_saw_orgp_wrong_side = True
                                elif self._hm_saw_orgp_wrong_side:
                                    self._hm_saw_orgp_wrong_side = False
                                    self._hm_search_dir = target_dir
                                    self._hm_slow_approach = True
                                    self._vel_demand_pps = 0.0
                                    self._vel_actual_pps = 0.0
                                    self._pos_demand_f = self._pos_actual_f
                                    skip_motion = True

            if not skip_motion and self._hm_phase == HomingPhase.SEARCH_SWITCH:
                spd = max(500.0, v_zero) if self._hm_slow_approach else max(1000.0, v_switch)
                target_v = self._hm_search_dir * spd
                self._vel_demand_pps = self._ramp_velocity(self._vel_demand_pps, target_v, accel, accel, dt)
                self._pos_demand_f += self._vel_demand_pps * dt
                self._vel_actual_pps = self._vel_demand_pps

        elif self._hm_phase == HomingPhase.SEARCH_ZERO:
            self._pp_target_reached = False
            method = self.od.get_int(0x6098, 0x00)
            skip_motion = False

            # 尋 Z / 脫離極限期間若朝向另一端極限碰撞則報錯停止
            if (lsp and self._hm_search_dir > 0) or (lsn and self._hm_search_dir < 0):
                lim_name = "LSP" if self._hm_search_dir > 0 else "LSN"
                self._enter_hm_error(f"Homing Method {method} Error: Hit {lim_name} during zero search")
                skip_motion = True

            if not skip_motion:
                v_zero = min(abs(self._vel_od_to_pps(self.od.get_int(0x6099, 0x02))), self._velocity_limit_pps(apply_2d20=False))
                accel = self._accel_od_to_pps2(self.od.get_int(0x609A, 0x00))
                target_v = self._hm_search_dir * max(500.0, v_zero)
                self._vel_demand_pps = self._ramp_velocity(self._vel_demand_pps, target_v, accel, accel, dt)
                prev_pos = self._pos_demand_f
                self._pos_demand_f += self._vel_demand_pps * dt
                self._vel_actual_pps = self._vel_demand_pps

                cmd_ppr = self._cmd_pulses_per_rev()
                z_crossed = int(math.floor(prev_pos / cmd_ppr)) != int(math.floor(self._pos_demand_f / cmd_ppr))
                z_hit = self.sensors.z_phase or z_crossed

                if method == 17:
                    if not lsn:
                        self._finish_homing()
                elif method == 18:
                    if not lsp:
                        self._finish_homing()
                elif method == 1:
                    if (not lsn) and z_hit:
                        self._finish_homing()
                elif method == 2:
                    if (not lsp) and z_hit:
                        self._finish_homing()
                else:
                    if z_hit:
                        self._finish_homing()

        self._hm_prev_lsn = lsn
        self._hm_prev_lsp = lsp
        self._hm_prev_orgp = orgp

    def _finish_homing(self, event_msg: Optional[str] = None) -> None:
        offset = self.od.get_int(0x607C, 0x00)
        self._pos_demand_f = float(offset)
        self._pos_actual_f = float(offset)
        self._pp_target_pos = offset
        self._vel_demand_pps = 0.0
        self._vel_actual_pps = 0.0
        self._torque_actual_01pct = 0.0
        self._hm_phase = HomingPhase.DONE
        self._hm_attained = True
        self._hm_error = False
        self._hm_waiting_orgp_fall = False
        self._hm_need_leave_orgp = False
        self._hm_saw_orgp_wrong_side = False
        self._hm_slow_approach = False
        self._pp_target_reached = True
        self._hm_prev_rev = int(math.floor(self._pos_actual_f / self._cmd_pulses_per_rev()))
        if self.on_event:
            self.on_event(
                self.axis_id,
                event_msg or f"Homing Completed -> Actual Position = {offset} pulse",
            )

    def _step_pv_mode(self, cw: int, dt: float) -> None:
        target_pps, limit_pps = self._pv_target_pps(cw)
        lsn, lsp, _ = self._effective_sensors()
        sw_min = float(self.od.get_int(0x607D, 0x01))
        sw_max = float(self.od.get_int(0x607D, 0x02))

        # 若正極限 (LSP) 或軟體上限動作，立即凍結正方向速度與位置；負極限 (LSN) 同理
        blocked_pos = (lsp or self._pos_demand_f >= sw_max) and (target_pps > 0 or self._vel_demand_pps > 0)
        blocked_neg = (lsn or self._pos_demand_f <= sw_min) and (target_pps < 0 or self._vel_demand_pps < 0)

        if blocked_pos or blocked_neg:
            self._vel_demand_pps = 0.0
            self._vel_actual_pps = 0.0
            self._pos_actual_f = min(max(self._pos_actual_f, sw_min), sw_max)
            self._pos_demand_f = self._pos_actual_f
        else:
            accel = self._accel_od_to_pps2(self.od.get_int(0x6083, 0x00))
            decel = self._accel_od_to_pps2(self.od.get_int(0x6084, 0x00))

            prev_pos = self._pos_demand_f
            self._vel_demand_pps = self._ramp_velocity(self._vel_demand_pps, target_pps, accel, decel, dt)
            next_pos = prev_pos + self._vel_demand_pps * dt
            next_pos, self._vel_demand_pps, clamped = self._clamp_limits(prev_pos, next_pos, self._vel_demand_pps)
            if clamped and (lsp or lsn):
                self._pos_actual_f = min(max(self._pos_actual_f, sw_min), sw_max)
                next_pos = self._pos_actual_f
            self._pos_demand_f = next_pos
            self._vel_actual_pps = self._vel_demand_pps
        tol = max(self._pv_window_pps(), abs(target_pps) * 0.02)
        self._pp_target_reached = abs(self._vel_actual_pps - target_pps) <= tol
        self._torque_actual_01pct = self._estimate_torque_from_vel(self._vel_actual_pps, limit_pps)

    def _step_pt_mode(self, cw: int, dt: float) -> None:
        halt = bool(cw & 0x0100)
        pos_lim = self.od.get_int(0x60E0, 0x00)
        neg_lim = self.od.get_int(0x60E1, 0x00)
        raw_torque = 0.0 if halt else float(self.od.get_int(0x6071, 0x00))
        cmd_torque = max(-float(neg_lim), min(float(pos_lim), raw_torque))
        self._torque_limiting = (not halt) and (
            raw_torque > float(pos_lim) or raw_torque < -float(neg_lim)
        )

        slope = max(10.0, float(self.od.get_int(0x6087, 0x00)))
        max_step = slope * dt
        if abs(cmd_torque - self._torque_actual_01pct) <= max_step:
            self._torque_actual_01pct = cmd_torque
        else:
            self._torque_actual_01pct += math.copysign(max_step, cmd_torque - self._torque_actual_01pct)

        limit_pps = self._velocity_limit_pps(apply_2d20=True)
        cmd_ppr = self._cmd_pulses_per_rev()
        torque_accel = (self._torque_actual_01pct / 1000.0) * 20.0 * cmd_ppr
        viscous_drag = self._vel_actual_pps * 1.5
        unclamped_vel = self._vel_actual_pps + (torque_accel - viscous_drag) * dt
        self._vel_limiting = abs(unclamped_vel) >= limit_pps and limit_pps > 0.0
        self._vel_actual_pps = max(-limit_pps, min(limit_pps, unclamped_vel))
        self._vel_demand_pps = self._vel_actual_pps

        prev_pos = self._pos_actual_f
        next_pos = prev_pos + self._vel_actual_pps * dt
        next_pos, self._vel_actual_pps, clamped = self._clamp_limits(prev_pos, next_pos, self._vel_actual_pps)
        if clamped:
            self._vel_demand_pps = self._vel_actual_pps
        self._pos_actual_f = next_pos
        self._pos_demand_f = next_pos
        self._pp_target_reached = abs(self._torque_actual_01pct - cmd_torque) < 5.0

    def _step_quick_stop(self, dt: float) -> None:
        decel = self._accel_od_to_pps2(self.od.get_int(0x6085, 0x00))
        prev_pos = self._pos_demand_f
        self._vel_demand_pps = self._approach_zero(self._vel_demand_pps, decel, dt)
        next_pos = prev_pos + self._vel_demand_pps * dt
        next_pos, self._vel_demand_pps, _ = self._clamp_limits(prev_pos, next_pos, self._vel_demand_pps)
        self._pos_demand_f = next_pos
        self._vel_actual_pps = self._vel_demand_pps
        self._torque_actual_01pct = 0.0
        self._torque_limiting = False
        self._vel_limiting = False

    def _coast_to_stop(self, dt: float) -> None:
        decel = self._accel_od_to_pps2(self.od.get_int(0x6084, 0x00)) * 2.0
        prev_pos = self._pos_actual_f
        self._vel_actual_pps = self._approach_zero(self._vel_actual_pps, decel, dt)
        next_pos = prev_pos + self._vel_actual_pps * dt
        next_pos, self._vel_actual_pps, _ = self._clamp_limits(prev_pos, next_pos, self._vel_actual_pps)
        self._vel_demand_pps = self._vel_actual_pps
        self._pos_actual_f = next_pos
        self._pos_demand_f = self._pos_actual_f
        self._torque_actual_01pct = 0.0
        self._torque_limiting = False
        self._vel_limiting = False

    @staticmethod
    def _approach_zero(val: float, decel: float, dt: float) -> float:
        step = decel * dt
        if abs(val) <= step:
            return 0.0
        return val - math.copysign(step, val)

    @staticmethod
    def _ramp_velocity(current: float, target: float, accel: float, decel: float, dt: float) -> float:
        diff = target - current
        if abs(diff) < 1e-6:
            return target
        is_accel = (current * target >= 0) and (abs(target) > abs(current))
        rate = accel if is_accel else decel
        step = rate * dt
        if abs(diff) <= step:
            return target
        return current + math.copysign(step, diff)

    @staticmethod
    def _estimate_torque_from_vel(vel_pps: float, limit_pps: float) -> float:
        if abs(vel_pps) < 1.0:
            return 0.0
        ratio = vel_pps / max(1.0, limit_pps)
        return max(-1000.0, min(1000.0, ratio * 250.0))

    def _update_status_objects(self) -> None:
        base_sw = {
            PDSState.NOT_READY_TO_SWITCH_ON: 0x0000,
            PDSState.SWITCH_ON_DISABLED: 0x0070,
            PDSState.READY_TO_SWITCH_ON: 0x0031,
            PDSState.SWITCHED_ON: 0x0033,
            PDSState.OPERATION_ENABLED: 0x0037,
            PDSState.QUICK_STOP_ACTIVE: 0x0017,
            PDSState.FAULT_REACTION_ACTIVE: 0x001F,
            PDSState.FAULT: 0x0018,
        }[self.pds_state]
        if self.csr_active and self.pds_state != PDSState.NOT_READY_TO_SWITCH_ON:
            base_sw |= 0x0200  # Bit 9: Remote (Fieldbus control active)

        active_mode = self.od.get_int(0x6061, 0x00)
        if self.pds_state == PDSState.OPERATION_ENABLED:
            if active_mode != OpMode.NO_MODE and self._pp_target_reached:
                base_sw |= 0x0400
            if active_mode == OpMode.PP:
                if self._pp_sp_ack:
                    base_sw |= 0x1000
            elif active_mode == OpMode.HM:
                if self._hm_attained:
                    base_sw |= 0x1000
                if self._hm_error:
                    base_sw |= 0x2000
            elif active_mode == OpMode.PV:
                if abs(self._vel_actual_pps) <= self._zero_speed_pps():
                    base_sw |= 0x1000

        lsn, lsp, orgp = self._effective_sensors()
        if lsn or lsp:
            base_sw |= 0x0800

        self.od.set_internal(0x6041, 0x00, base_sw)

        demand_i = int(round(self._pos_demand_f))
        actual_i = int(round(self._pos_actual_f))
        internal_i = max(-2147483648, min(2147483647, int(round(self._pos_actual_f * self._gear_ratio()))))
        f_err = demand_i - actual_i

        self.od.set_internal(0x6062, 0x00, demand_i)
        self.od.set_internal(0x6063, 0x00, internal_i)
        self.od.set_internal(0x6064, 0x00, actual_i)
        self.od.set_internal(0x60F4, 0x00, f_err)
        self.od.set_internal(0x606B, 0x00, self._pps_to_vel_od(self._vel_demand_pps))
        self.od.set_internal(0x606C, 0x00, self._pps_to_vel_od(self._vel_actual_pps))
        self.od.set_internal(0x6077, 0x00, int(round(self._torque_actual_01pct)))

        di = 0
        if lsn:
            di |= 0x0001
        if lsp:
            di |= 0x0002
        if orgp:
            di |= 0x0004
        self.od.set_internal(0x60FD, 0x00, di)

        # 2D11h (Status DO 1) verified against physical SDC-F + MR-J5-G captures:
        #   - Servo OFF:                        0x0000
        #   - Servo ON Idle (No Mode / PP / HM): 0x1000 (Bit 12: S_INP)
        #   - Servo ON PV at target speed:      0x0004 (Bit 2: S_SA)
        #   - Servo ON PT torque limiting:      0x2000 (Bit 13: S_TLC)
        d01 = 0
        if self.pds_state == PDSState.OPERATION_ENABLED:
            cw_now = self.od.get_int(0x6040, 0x00)
            is_halted = bool(cw_now & 0x0100)
            target_vel_cmd = self.od.get_int(0x60FF, 0x00)
            if active_mode == OpMode.PV and (not is_halted) and target_vel_cmd != 0 and self._pp_target_reached:
                d01 |= 0x0004  # Bit 2: S_SA
            if (
                active_mode == OpMode.NO_MODE
                or (active_mode in (OpMode.PP, OpMode.HM) and (not self._pp_active) and self._pp_target_reached)
            ):
                d01 |= 0x1000  # Bit 12: S_INP
            if active_mode == OpMode.PT and self._torque_limiting:
                d01 |= 0x2000  # Bit 13: S_TLC
        self.od.set_internal(0x2D11, 0x00, d01)

        # 2D12h (Status DO 2) verified against physical SDC-F captures:
        #   - Zero speed (|vel| <= 606Fh): 0x0008 (Bit 3: S_ZSP)
        #   - Accelerating / Moving > ZSP: 0x0000
        #   - PT speed limiting:           0x0010 (Bit 4: S_VLC)
        d02 = 0
        if abs(self._vel_actual_pps) <= self._zero_speed_pps():
            d02 |= 0x0008  # Bit 3: S_ZSP
        if self.pds_state == PDSState.OPERATION_ENABLED and active_mode == OpMode.PT and self._vel_limiting:
            d02 |= 0x0010  # Bit 4: S_VLC
        self.od.set_internal(0x2D12, 0x00, d02)

    def get_snapshot(self) -> Dict[str, object]:
        lsn, lsp, orgp = self._effective_sensors()
        op_mode_id = self.od.get_int(0x6061, 0x00)
        return {
            "axis_id": self.axis_id,
            "ip": self.ip_address,
            "pds_state": self.pds_state.label,
            "op_mode": OpMode(op_mode_id).label
            if op_mode_id in (0, 1, 3, 4, 6)
            else str(op_mode_id),
            "op_mode_id": op_mode_id,
            "encoder_ppr": int(round(self._encoder_ppr())),
            "gear_num": self.od.get_int(0x2006, 0x00),
            "gear_den": self.od.get_int(0x2007, 0x00),
            "csr": self.csr_active,
            "ssr": self.ssr_active,
            "controlword": self.od.get_int(0x6040, 0x00),
            "statusword": self.od.get_int(0x6041, 0x00),
            "pos_demand": self.od.get_int(0x6062, 0x00),
            "pos_actual": self.od.get_int(0x6064, 0x00),
            "following_error": self.od.get_int(0x60F4, 0x00),
            "vel_actual": self.od.get_int(0x606C, 0x00),
            "vel_limit": self.od.get_int(0x2D20, 0x00),
            "torque_actual": self.od.get_int(0x6077, 0x00),
            "error_code": self.od.get_int(0x603F, 0x00),
            "homing_method": self.od.get_int(0x6098, 0x00),
            "angle_deg": self._motor_angle_deg,
            "lsn": lsn,
            "lsp": lsp,
            "orgp": orgp,
            "rx_count": self.cyclic_rx_count,
            "tx_count": self.cyclic_tx_count,
        }
