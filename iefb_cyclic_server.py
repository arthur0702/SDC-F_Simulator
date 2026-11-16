"""
CC-Link IE Field Basic (IEFB) 循環通訊與探索伺服器模組
====================================================

負責處理士林電機 SDC-F 模擬器與 AX5S PLC 之間的底層 UDP 通訊：
- UDP 61450：SLMP 3E/4E 0E70h CyclicData 循環資料交換（131B 實機 SDC-F 回應格式）
- UDP 61451：NodeSearch (0E30h) 站台探索與 IPAddressSet (0E31h) IP 設定
- UDP 45237：iQSS 探索與管理回應
- MultiAxisCyclicManager：多軸靜態 IP 綁定、0.0.0.0 廣播接聽分流與 30ms 跨 Socket 去重
- 輔助診斷工具：本機路由檢查與 Windows 多 IP / 防火牆 netsh 腳本產生器
"""

from __future__ import annotations

import asyncio
import ipaddress
import socket
import struct
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

from object_dictionary import (
    CCIEF_PROTOCOL_VERSION,
    CSP_VENDOR_CODE,
    DEVICE_MODEL,
    DEVICE_TYPE_ID,
    OCCUPIED_STATIONS,
    PRODUCT_ID,
    VENDOR_NAME,
)
from servo_axis import ServoAxis
from slmp_server import SLMPFrameProcessor, SLMPServer, SLMPTransaction


IEFB_CYCLIC_PORT = 61450
IEFB_NODESEARCH_PORT = 61451
IQSS_DISCOVERY_PORT = 45237

# Each occupied station in CC-Link IEFB uses:
# RX/RY = 64 bits (8 bytes), RWr/RWw = 32 words (64 bytes) -> 72 bytes total per direction
STATION_BIT_BYTES = 8
STATION_WORD_BYTES = 64
STATION_TOTAL_BYTES = STATION_BIT_BYTES + STATION_WORD_BYTES  # 72 bytes


class CyclicPackingLayout(str, Enum):
    """Defines how multi-station RY/RWw data is arranged inside a UDP 61450 packet."""

    INTERLEAVED = "interleaved"  # [Header][RY_0..RY_(N-1)][RWw_0..RWw_(N-1)] (CLPA Standard)
    PER_STATION = "per_station"  # [Header][RY_0 + RWw_0]...[RY_(N-1) + RWw_(N-1)]


@dataclass
class CyclicFrameHeader:
    """Parsed CC-Link IE Field Basic Cyclic UDP 61450 Header."""

    protocol_version: int = CCIEF_PROTOCOL_VERSION
    end_code: int = 0x0000
    master_id: int = 0x00000000
    master_timestamp: int = 0
    group_no: int = 0
    frame_seq: int = 0
    cyclic_state: int = 0x0001
    total_stations: int = 1
    header_len: int = 24
    raw_header: bytes = b""
    is_slmp_0e70: bool = False
    slmp_frame_type: str = ""
    slmp_serial: bytes = b""
    slmp_prefix: bytes = b"\x00\xff\xff\x03\x00"
    slmp_cmd: int = 0x0E70
    slmp_subcmd: int = 0x0000
    slmp_0e70_offset: int = 36
    master_info_raw: bytes = b""
    cyclic_info_raw: bytes = b""



# 實機 AX5S + SDC-F 已驗證之 SLMP 0E70h 回應格式 (131B = 11B SLMP + 48B IEFB 標頭 + 64B RWr + 8B RX)
SLMP_0E70_PROBE_ORDER: Tuple[str, ...] = (
    "slmp11_real_sdcf_48b_wfirst",
    "slmp11_real_sdcf_48b",
)


@dataclass
class CyclicFrameConfig:
    """CC-Link IE Field Basic 循環通訊封包設定（支援自動偵測 0E70h / Ver 1 / Ver 2 標頭）。"""

    auto_detect: bool = True
    ry_offset: int = 24
    rww_offset: int = 32
    station_index_in_frame: int = 0
    packing_layout: CyclicPackingLayout = CyclicPackingLayout.INTERLEAVED
    timeout_ms: int = 30000  # 依 CSP+ XML TimeOutValue = 30000 ms
    slmp_resp_profile: str = "slmp11_real_sdcf_48b_wfirst"


@dataclass
class CyclicFrameStats:
    """單軸 UDP 61450 循環通訊即時統計與連線狀態。"""

    rx_packets: int = 0
    tx_packets: int = 0
    malformed_packets: int = 0
    last_seq_no: int = -1
    dropped_seq_count: int = 0
    last_rx_ts: float = 0.0
    last_scan_ms: float = 0.0
    avg_scan_ms: float = 0.0
    master_ip: str = ""
    master_port: int = 0
    last_header_len: int = 0
    last_pkt_len: int = 0
    last_protocol_ver: int = CCIEF_PROTOCOL_VERSION
    last_is_slmp_0e70: bool = False
    timeout_active: bool = False
    link_lost: bool = False
    active_resp_profile: str = ""
    locked_resp_profile: str = ""
    probe_index: int = 0
    probe_packet_hits: int = 0
    fast_rate_hold_hits: int = 0
    csr_high_since: float = 0.0
    sdcf_body_order_idx: int = 0
    csr_switch_count: int = 0
    logged_shape: Optional[Tuple[int, int, bool]] = None
    last_pdo_payload: Optional[Tuple[bytes, bytes]] = None
    last_pdo_log_ts: float = 0.0

    def idle_seconds(self, now: Optional[float] = None) -> float:
        """計算距離上次收到主站封包經過的秒數。"""
        if self.last_rx_ts <= 0.0:
            return 0.0
        cur = time.perf_counter() if now is None else now
        return max(0.0, cur - self.last_rx_ts)

    def is_link_alive(self, now: Optional[float] = None, link_timeout_s: float = 1.5) -> bool:
        """判斷循環通訊是否仍在線（未超過逾時時間）。"""
        if self.last_rx_ts <= 0.0 or self.link_lost:
            return False
        return self.idle_seconds(now) <= link_timeout_s

    def reset_probe_state(self) -> None:
        """重置回應格式偵測與握手追蹤狀態。"""
        self.active_resp_profile = ""
        self.locked_resp_profile = ""
        self.probe_index = 0
        self.probe_packet_hits = 0
        self.fast_rate_hold_hits = 0
        self.csr_high_since = 0.0
        self.sdcf_body_order_idx = 0
        self.csr_switch_count = 0
        self.logged_shape = None
        self.last_pdo_payload = None
        self.last_pdo_log_ts = 0.0


class CyclicFrameCodec:
    """
    CC-Link IE Field Basic 循環封包編解碼器（UDP Port 61450）：
    - 解析 AX5S SLMP 3E/4E Binary Command 0E70h CyclicData 請求（[RWw 64B*N][RY 8B*N]）
    - 組裝與實機 SDC-F 1:1 一致之 131B 0E70h 回應封包（[RWr 64B][RX 8B]）
    - 相容傳統 Ver 1 / Ver 2 及極簡 72B 測試封包
    """

    @staticmethod
    def detect_slmp_0e70(data: bytes) -> Optional[Tuple[str, int, bytes, bytes, int, int]]:
        """
        Detect SLMP 3E (50 00) or 4E (54 00) CyclicData (0E70h) frame.
        Returns (frame_type, slmp_hdr_len, serial_prefix, routing_prefix, cmd, subcmd) or None.
        """
        pkt_len = len(data)
        if pkt_len >= 15 and data[0:2] == b"\x50\x00":
            cmd, subcmd = struct.unpack_from("<HH", data, 11)
            if cmd == 0x0E70:
                return ("3E", 15, b"", data[2:7], cmd, subcmd)
        if pkt_len >= 19 and data[0:2] == b"\x54\x00":
            cmd, subcmd = struct.unpack_from("<HH", data, 15)
            if cmd == 0x0E70:
                return ("4E", 19, data[2:6], data[6:11], cmd, subcmd)
        return None

    @classmethod
    def parse_header(cls, data: bytes, header_len: int) -> CyclicFrameHeader:
        if header_len <= 0:
            return CyclicFrameHeader(header_len=0, raw_header=b"")

        raw_hdr = data[:header_len]
        slmp_info = cls.detect_slmp_0e70(data)
        if slmp_info is not None:
            ftype, slmp_hdr_len, serial_prefix, routing_prefix, cmd, subcmd = slmp_info
            p = slmp_hdr_len
            ver = struct.unpack_from("<H", data, p)[0] if header_len >= p + 2 else CCIEF_PROTOCOL_VERSION
            if ver not in (1, 2):
                ver = CCIEF_PROTOCOL_VERSION
            end_code = struct.unpack_from("<H", data, p + 2)[0] if header_len >= p + 4 else 0
            off_val = struct.unpack_from("<I", data, p + 4)[0] if header_len >= p + 8 else 36
            if off_val < 20 or off_val > 256:
                off_val = 36

            master_info_raw = data[p + 20 : p + off_val] if header_len >= p + off_val else b""
            master_ts = (
                struct.unpack_from("<I", data, p + 24)[0]
                if off_val >= 36 and header_len >= p + 28
                else 0
            )
            master_id = (
                struct.unpack_from("<I", data, p + 32)[0]
                if off_val >= 36 and header_len >= p + 36
                else (struct.unpack_from("<I", data, p + 2)[0] if header_len >= p + 6 else 0)
            )

            c = p + off_val
            if header_len >= c + 16:
                cyclic_info_raw = data[c : c + 16]
                group_no = struct.unpack_from("<H", data, c)[0]
                frame_seq = struct.unpack_from("<H", data, c + 2)[0]
                total_sta = struct.unpack_from("<H", data, c + 10)[0]
                if total_sta == 0 or total_sta > 64:
                    total_sta = 1
            elif header_len >= c + 4:
                cyclic_info_raw = data[c : header_len]
                group_no = struct.unpack_from("<H", data, c)[0]
                frame_seq = struct.unpack_from("<H", data, c + 2)[0]
                total_sta = 1
            else:
                cyclic_info_raw = b""
                group_no = data[p + 6] if header_len >= p + 7 else 0
                frame_seq = struct.unpack_from("<H", data, p + 8)[0] if header_len >= p + 10 else 0
                total_sta = 1

            return CyclicFrameHeader(
                protocol_version=ver,
                end_code=end_code,
                master_id=master_id,
                master_timestamp=master_ts,
                group_no=group_no,
                frame_seq=frame_seq,
                cyclic_state=0x0001,
                total_stations=total_sta,
                header_len=header_len,
                raw_header=raw_hdr,
                is_slmp_0e70=True,
                slmp_frame_type=ftype,
                slmp_serial=serial_prefix,
                slmp_prefix=routing_prefix,
                slmp_cmd=cmd,
                slmp_subcmd=subcmd,
                slmp_0e70_offset=off_val,
                master_info_raw=master_info_raw,
                cyclic_info_raw=cyclic_info_raw,
            )

        ver = struct.unpack_from("<H", raw_hdr, 0)[0] if header_len >= 2 else CCIEF_PROTOCOL_VERSION
        end_code = struct.unpack_from("<H", raw_hdr, 2)[0] if header_len >= 4 else 0
        master_id = struct.unpack_from("<I", raw_hdr, 4)[0] if header_len >= 8 else 0
        group_no = struct.unpack_from("<H", raw_hdr, 8)[0] if header_len >= 10 else 0
        frame_seq = struct.unpack_from("<H", raw_hdr, 10)[0] if header_len >= 12 else 0
        cyclic_state = struct.unpack_from("<H", raw_hdr, 12)[0] if header_len >= 14 else 1
        total_sta = struct.unpack_from("<H", raw_hdr, 14)[0] if header_len >= 16 else 1
        if total_sta == 0 or total_sta > 64:
            total_sta = 1

        return CyclicFrameHeader(
            protocol_version=ver,
            end_code=end_code,
            master_id=master_id,
            group_no=group_no,
            frame_seq=frame_seq,
            cyclic_state=cyclic_state,
            total_stations=total_sta,
            header_len=header_len,
            raw_header=raw_hdr,
        )

    @classmethod
    def extract_ry_rww(
        cls,
        data: bytes,
        axis: ServoAxis,
        cfg: CyclicFrameConfig,
        station_override_idx: Optional[int] = None,
    ) -> Optional[Tuple[bytes, bytes, CyclicFrameHeader]]:
        """
        Parse incoming CC-Link IEFB UDP 61450 datagram for a given axis.
        Returns (ry_8bytes, rww_64bytes, parsed_header) or None if invalid.
        """
        pkt_len = len(data)
        if pkt_len < STATION_TOTAL_BYTES:
            return None

        if not cfg.auto_detect:
            ry_off = cfg.ry_offset
            rww_off = cfg.rww_offset
            if ry_off < 0 or rww_off < 0:
                return None
            if ry_off + STATION_BIT_BYTES > pkt_len or rww_off + STATION_WORD_BYTES > pkt_len:
                return None
            hdr = cls.parse_header(data, min(ry_off, rww_off))
            return (
                data[ry_off : ry_off + STATION_BIT_BYTES],
                data[rww_off : rww_off + STATION_WORD_BYTES],
                hdr,
            )

        # Minimal 72-byte headerless frame: [RY(8B)][RWw(64B)]
        if pkt_len == STATION_TOTAL_BYTES:
            hdr = CyclicFrameHeader(header_len=0, total_stations=1, raw_header=b"")
            return data[0:8], data[8:72], hdr

        # Check for real SLMP 3E/4E CyclicData (0E70h) frame from AX5S PLC
        slmp_info = cls.detect_slmp_0e70(data)
        if slmp_info is not None:
            slmp_hdr_len = slmp_info[1]
            p = slmp_hdr_len
            off_val = struct.unpack_from("<I", data, p + 4)[0] if pkt_len >= p + 8 else 36
            if off_val < 20 or off_val > 256:
                off_val = 36

            ip_table_start = p + off_val + 16
            total_stations = 0
            header_len = 0

            # Exact AX5S 0E70h Ver 2 layout: Read TotalSlaves at p+off_val+10 (byte 61..62)
            # Header = 15 (SLMP) + off_val (36) + 16 (CyclicInfo) + 4*N (SlaveIPs) = 67 + 4*N
            if pkt_len >= ip_table_start + STATION_TOTAL_BYTES:
                n_slaves = struct.unpack_from("<H", data, p + off_val + 10)[0]
                if 1 <= n_slaves <= 64:
                    expected_hdr = ip_table_start + (4 * n_slaves)
                    if pkt_len == expected_hdr + (n_slaves * STATION_TOTAL_BYTES):
                        total_stations = n_slaves
                        header_len = expected_hdr

            if total_stations == 0:
                if pkt_len >= slmp_hdr_len + 52 + STATION_TOTAL_BYTES:
                    min_hdr = slmp_hdr_len + 52
                elif pkt_len >= slmp_hdr_len + 16 + STATION_TOTAL_BYTES:
                    min_hdr = slmp_hdr_len + 16
                else:
                    min_hdr = slmp_hdr_len

                if pkt_len < min_hdr + STATION_TOTAL_BYTES:
                    return None

                total_stations = max(1, (pkt_len - min_hdr) // STATION_TOTAL_BYTES)
                header_len = pkt_len - (total_stations * STATION_TOTAL_BYTES)

            hdr = cls.parse_header(data, header_len)
            hdr.total_stations = total_stations

            # 優先依據 0E70h 封包內含的 Slave IP 表 (4B LE x N) 精確比對此軸 IP；
            # 若為保留站 (0.0.0.0) 或該軸 IP 不在 IP 表內則靜默忽略不回應
            has_ip_table = header_len == ip_table_start + (4 * total_stations)
            ip_matched_idx: Optional[int] = None
            if has_ip_table:
                axis_ip_le = ipaddress.IPv4Address(axis.ip_address).packed[::-1]
                for idx in range(total_stations):
                    off_ip = ip_table_start + (idx * 4)
                    if data[off_ip : off_ip + 4] == axis_ip_le:
                        ip_matched_idx = idx
                        break
                if ip_matched_idx is None:
                    return None

            if ip_matched_idx is not None:
                sta_idx = ip_matched_idx
            elif station_override_idx is not None:
                sta_idx = max(0, min(total_stations - 1, station_override_idx))
            elif total_stations > 1:
                sta_idx = max(0, min(total_stations - 1, axis.station_no - 1))
            else:
                sta_idx = max(0, min(total_stations - 1, cfg.station_index_in_frame))

            if cfg.packing_layout == CyclicPackingLayout.PER_STATION:
                base = header_len + (sta_idx * STATION_TOTAL_BYTES)
                ry_off = base
                rww_off = base + STATION_BIT_BYTES
            else:
                # Proven from AX5S SLMP 0E70h capture: [RWw (64B * N)][RY (8B * N)]
                # (@71..134 = RWw, @135..142 = RY where @142 MSB 0x80 = RY3F CSR)
                rww_off = header_len + (sta_idx * STATION_WORD_BYTES)
                ry_off = header_len + (total_stations * STATION_WORD_BYTES) + (sta_idx * STATION_BIT_BYTES)

            if ry_off + STATION_BIT_BYTES > pkt_len or rww_off + STATION_WORD_BYTES > pkt_len:
                return None

            return (
                data[ry_off : ry_off + STATION_BIT_BYTES],
                data[rww_off : rww_off + STATION_WORD_BYTES],
                hdr,
            )

        # Legacy / synthetic non-SLMP CC-Link IEFB cyclic frame (12..64 byte header)
        total_stations = max(1, pkt_len // STATION_TOTAL_BYTES)
        header_len = pkt_len - (total_stations * STATION_TOTAL_BYTES)

        if header_len < 12 and total_stations > 1:
            total_stations -= 1
            header_len = pkt_len - (total_stations * STATION_TOTAL_BYTES)

        if header_len < 0 or header_len > 64:
            ry_off = min(cfg.ry_offset, max(0, pkt_len - STATION_TOTAL_BYTES))
            rww_off = ry_off + STATION_BIT_BYTES
            hdr = cls.parse_header(data, ry_off)
            return (
                data[ry_off : ry_off + STATION_BIT_BYTES],
                data[rww_off : rww_off + STATION_WORD_BYTES],
                hdr,
            )

        hdr = cls.parse_header(data, header_len)
        hdr.total_stations = total_stations

        if station_override_idx is not None:
            sta_idx = station_override_idx
        elif total_stations > 1:
            sta_idx = max(0, min(total_stations - 1, axis.station_no - 1))
        else:
            sta_idx = max(0, min(total_stations - 1, cfg.station_index_in_frame))

        if cfg.packing_layout == CyclicPackingLayout.PER_STATION:
            base = header_len + (sta_idx * STATION_TOTAL_BYTES)
            ry_off = base
            rww_off = base + STATION_BIT_BYTES
        else:
            # CLPA Standard INTERLEAVED: all RY blocks first, then all RWw blocks
            ry_off = header_len + (sta_idx * STATION_BIT_BYTES)
            rww_off = header_len + (total_stations * STATION_BIT_BYTES) + (sta_idx * STATION_WORD_BYTES)

        if ry_off + STATION_BIT_BYTES > pkt_len or rww_off + STATION_WORD_BYTES > pkt_len:
            return None

        return (
            data[ry_off : ry_off + STATION_BIT_BYTES],
            data[rww_off : rww_off + STATION_WORD_BYTES],
            hdr,
        )

    @staticmethod
    def build_master_request(
        ry_list: List[bytes],
        rww_list: List[bytes],
        frame_seq: int = 1,
        protocol_version: int = CCIEF_PROTOCOL_VERSION,
        master_ip: str = "192.168.3.250",
        group_no: int = 0,
        header_len: int = 24,
        packing_layout: CyclicPackingLayout = CyclicPackingLayout.INTERLEAVED,
    ) -> bytes:
        """Build a legacy/compact CC-Link IEFB Master -> Slave Cyclic request frame."""
        n_sta = max(len(ry_list), len(rww_list), 1)
        norm_ry = [
            (ry_list[i] if i < len(ry_list) else b"\x00" * 8).ljust(8, b"\x00")[:8]
            for i in range(n_sta)
        ]
        norm_rww = [
            (rww_list[i] if i < len(rww_list) else b"\x00" * 64).ljust(64, b"\x00")[:64]
            for i in range(n_sta)
        ]

        hdr = bytearray(header_len)
        if header_len >= 2:
            struct.pack_into("<H", hdr, 0, protocol_version)
        if header_len >= 4:
            struct.pack_into("<H", hdr, 2, 0x0000)
        if header_len >= 8:
            hdr[4:8] = ipaddress.IPv4Address(master_ip).packed
        if header_len >= 10:
            struct.pack_into("<H", hdr, 8, group_no)
        if header_len >= 12:
            struct.pack_into("<H", hdr, 10, frame_seq & 0xFFFF)
        if header_len >= 14:
            struct.pack_into("<H", hdr, 12, 0x0001)  # Cyclic RUN
        if header_len >= 16:
            struct.pack_into("<H", hdr, 14, n_sta)

        if packing_layout == CyclicPackingLayout.PER_STATION:
            body = b"".join(norm_ry[i] + norm_rww[i] for i in range(n_sta))
        else:
            body = b"".join(norm_ry) + b"".join(norm_rww)

        return bytes(hdr) + body

    @staticmethod
    def build_slmp_0e70_request(
        ry_list: List[bytes],
        rww_list: List[bytes],
        frame_seq: int = 1,
        protocol_version: int = CCIEF_PROTOCOL_VERSION,
        master_ip: str = "192.168.3.250",
        group_no: int = 1,
        slave_ips: Optional[List[str]] = None,
        scan_interval_ms: int = 500,
        timeout_count: int = 3,
        packing_layout: CyclicPackingLayout = CyclicPackingLayout.INTERLEAVED,
        master_timestamp: int = 0x2033FB25,
    ) -> bytes:
        """Build an AX5S SLMP 3E (50 00 ... 70 0E 00 00) CyclicData Request (Ver 2)."""
        n_sta = max(len(ry_list), len(rww_list), 1)
        norm_ry = [
            (ry_list[i] if i < len(ry_list) else b"\x00" * 8).ljust(8, b"\x00")[:8]
            for i in range(n_sta)
        ]
        norm_rww = [
            (rww_list[i] if i < len(rww_list) else b"\x00" * 64).ljust(64, b"\x00")[:64]
            for i in range(n_sta)
        ]

        fixed = bytearray(52)
        # Common Header (20B at bytes 15..34): ProtoVer(2), EndCode(2), Offset=36(4), Reserved(12)
        struct.pack_into("<HHI", fixed, 0, protocol_version, 0x0000, 36)
        # Master Info (16B at bytes 35..50): Flags(4), Timestamp(4), Param=0x50(4), MasterIP(4 LE)
        struct.pack_into("<III", fixed, 20, 1, master_timestamp & 0xFFFFFFFF, 0x00000050)
        fixed[32:36] = ipaddress.IPv4Address(master_ip).packed[::-1]
        # Cyclic Info (16B at bytes 51..66, Offset=36): GroupNo(2), FrameSeq(2), ScanMs(2), Retry(2), Wait(2), TotalSlaves(2), Rsv(4)
        struct.pack_into(
            "<HHHHHHI",
            fixed,
            36,
            group_no & 0xFFFF,
            frame_seq & 0xFFFF,
            scan_interval_ms & 0xFFFF,
            timeout_count & 0xFFFF,
            (scan_interval_ms + timeout_count) & 0xFFFF,
            n_sta & 0xFFFF,
            0,
        )

        # Slave IP Table (4B x n_sta at bytes 67..)
        ip_tbl = bytearray()
        for idx in range(n_sta):
            s_ip = (
                slave_ips[idx]
                if slave_ips and idx < len(slave_ips)
                else f"192.168.3.{idx + 1}"
            )
            ip_tbl.extend(ipaddress.IPv4Address(s_ip).packed[::-1])

        if packing_layout == CyclicPackingLayout.PER_STATION:
            cyclic_body = b"".join(norm_ry[i] + norm_rww[i] for i in range(n_sta))
        else:
            cyclic_body = b"".join(norm_rww) + b"".join(norm_ry)

        req_payload = bytes(fixed) + bytes(ip_tbl) + cyclic_body
        data_len = 6 + len(req_payload)
        slmp_hdr = struct.pack(
            "<HBBHBHHHH",
            0x0050,
            0x00,
            0xFF,
            0x03FF,
            0x00,
            data_len,
            0x0000,
            0x0E70,
            0x0000,
        )
        return slmp_hdr + req_payload

    @staticmethod
    def build_slmp_0e70_profile_response(
        profile: str,
        req_header: CyclicFrameHeader,
        axis: ServoAxis,
        rx_padded: bytes,
        rwr_padded: bytes,
    ) -> bytes:
        """
        依實機 AX5S + SDC-F 驗證格式組裝 131B SLMP 0E70h 從站回應封包：
        - SLMP 3E 回應標頭 (11B)：0x00D0 + Routing(5B) + DataLen(2B=0x007A) + EndCode(2B=0x0000)
        - IEFB 回應標頭 (48B)：ProtoVer(2B) + EndCode(2B) + Offset=40(4B) + Reserved(12B) +
          VendorInfo(20B: Vendor=0x0370, Product=0x000D, Type=1, Occ=1, 0) + SlaveIP_LE(4B) + Group(2B) + Seq(2B)
        - PDO 主體 (72B)：實機 wfirst 格式為 [RWr 64B][RX 8B]（與主站請求 [RWw 64B][RY 8B] 對稱）
        """
        resp_subhdr = 0x00D4 if req_header.slmp_frame_type == "4E" else 0x00D0
        ver = req_header.protocol_version or CCIEF_PROTOCOL_VERSION
        grp = req_header.group_no & 0xFFFF
        seq = req_header.frame_seq & 0xFFFF
        slave_ip_le = ipaddress.IPv4Address(axis.ip_address).packed[::-1]

        common_off40 = struct.pack("<HHI12s", ver, 0x0000, 40, b"\x00" * 12)
        real_sdcf_slave_info = struct.pack(
            "<IIIII",
            CSP_VENDOR_CODE,
            PRODUCT_ID,
            0x00000001,
            OCCUPIED_STATIONS,
            0x00000000,
        )
        real_sdcf_cyc_tail = slave_ip_le + struct.pack("<HH", grp, seq)
        real_sdcf_hdr48 = common_off40 + real_sdcf_slave_info + real_sdcf_cyc_tail

        # 預設採用實機驗證之 RWr 在前、RX 在後 (wfirst)；保留 slmp11_real_sdcf_48b 為 [RX][RWr] 對照
        word_first_body = profile != "slmp11_real_sdcf_48b"
        pdo_body = (rwr_padded + rx_padded) if word_first_body else (rx_padded + rwr_padded)
        ccief_body = real_sdcf_hdr48 + pdo_body

        payload_after_len = struct.pack("<H", 0x0000) + ccief_body
        routing = (req_header.slmp_prefix or b"\x00\xff\xff\x03\x00")[:5].ljust(5, b"\x00")
        serial = req_header.slmp_serial[:4] if req_header.slmp_frame_type == "4E" else b""
        return (
            struct.pack("<H", resp_subhdr)
            + serial
            + routing
            + struct.pack("<H", len(payload_after_len))
            + payload_after_len
        )

    @classmethod
    def build_response(
        cls,
        req_header: CyclicFrameHeader,
        axis: ServoAxis,
        rx_bytes: bytes,
        rwr_bytes: bytes,
        slmp_profile: str = "slmp11_real_sdcf_48b_wfirst",
    ) -> bytes:
        """Build Slave -> Master CC-Link IEFB Cyclic response frame (SLMP 0E70h Ver 2 or legacy)."""
        rx_padded = rx_bytes.ljust(STATION_BIT_BYTES, b"\x00")[:STATION_BIT_BYTES]
        rwr_padded = rwr_bytes.ljust(STATION_WORD_BYTES, b"\x00")[:STATION_WORD_BYTES]

        if req_header.is_slmp_0e70:
            prof = (
                SLMP_0E70_PROBE_ORDER[0]
                if slmp_profile in ("auto", "slmp11_real_sdcf_auto")
                else slmp_profile
            )
            return cls.build_slmp_0e70_profile_response(prof, req_header, axis, rx_padded, rwr_padded)

        header_len = req_header.header_len
        if header_len <= 0:
            return rx_padded + rwr_padded

        hdr = bytearray(header_len)
        copy_len = min(len(req_header.raw_header), header_len)
        hdr[:copy_len] = req_header.raw_header[:copy_len]

        if header_len >= 2:
            struct.pack_into("<H", hdr, 0, req_header.protocol_version or CCIEF_PROTOCOL_VERSION)
        if header_len >= 4:
            struct.pack_into("<H", hdr, 2, 0x0000)  # End Code = Normal (0x0000)
        if header_len >= 8:
            hdr[4:8] = ipaddress.IPv4Address(axis.ip_address).packed
        if header_len >= 12:
            struct.pack_into("<H", hdr, 10, req_header.frame_seq & 0xFFFF)
        if header_len >= 14:
            slave_state = 0x0001 if axis.ssr_active else 0x0000
            struct.pack_into("<H", hdr, 12, slave_state)
        if header_len >= 16:
            struct.pack_into("<H", hdr, 14, OCCUPIED_STATIONS)

        return bytes(hdr) + rx_padded + rwr_padded


class _IEFBCyclicProtocol(asyncio.DatagramProtocol):
    """UDP Port 61450 Datagram Protocol supporting single-axis or multi-axis station dispatch."""

    def __init__(
        self,
        axes_by_station: Dict[int, ServoAxis],
        cfg: CyclicFrameConfig,
        stats_by_axis: Dict[int, CyclicFrameStats],
        on_cyclic_packet: Optional[Callable[[int, CyclicFrameHeader], None]] = None,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
        reply_transport_fn: Optional[Callable[[ServoAxis], Optional[asyncio.DatagramTransport]]] = None,
        dedup_cache: Optional[Dict[int, Tuple[str, Tuple[str, int], bytes, float]]] = None,
        source_tag: str = "Cyclic-61450",
    ) -> None:
        self.axes_by_station = axes_by_station
        self.cfg = cfg
        self.stats_by_axis = stats_by_axis
        self.on_cyclic_packet = on_cyclic_packet
        self.on_network_diag = on_network_diag
        self.reply_transport_fn = reply_transport_fn
        self.dedup_cache = dedup_cache if dedup_cache is not None else {}
        self.source_tag = source_tag
        self.transport: Optional[asyncio.DatagramTransport] = None
        self._last_logged_shape: Dict[int, Tuple[int, int, bool]] = {}
        self._last_pdo_payload: Dict[int, Tuple[bytes, bytes]] = {}
        self._last_pdo_log_ts: Dict[int, float] = {}

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def error_received(self, exc: Exception) -> None:
        if self.on_network_diag is not None and self.axes_by_station:
            first_ax = next(iter(self.axes_by_station.values()))
            self.on_network_diag(
                first_ax.axis_id,
                f"[{self.source_tag} SOCKET ERR] UDP 送收異常: {exc!r}",
            )

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        now = time.perf_counter()

        # If bound to a single axis (1 station per IP), process directly for that axis
        if len(self.axes_by_station) == 1:
            axis = next(iter(self.axes_by_station.values()))
            self._process_for_axis(axis, data, addr, now, station_override_idx=None)
            return

        # Shared multi-axis multiplexer / broadcast listener on 0.0.0.0:61450
        for idx, sta_no in enumerate(sorted(self.axes_by_station.keys())):
            axis = self.axes_by_station[sta_no]
            self._process_for_axis(axis, data, addr, now, station_override_idx=idx)

    def _select_slmp_resp_profile(
        self,
        stats: CyclicFrameStats,
        axis: ServoAxis,
        ry_bytes: bytes,
        rww_bytes: bytes,
        scan_ms: float,
        now: float,
    ) -> Tuple[str, str]:
        """
        Select SLMP 0E70h response profile.
        - 'slmp11_real_sdcf_auto': uses the verified 131B SDC-F header, starting with
          [RWr(64B)][RX(8B)] ('wfirst') symmetric to AX5S [RWw(64B)][RY(8B)], and watches
          CSR (RY3F): if AX5S drops CSR after ~5s SSR timeout, auto-flips to [RX][RWr].
          Locks permanently once CSR holds >= 6.5s or user RY data arrives.
        """
        cfg_prof = getattr(self.cfg, "slmp_resp_profile", "slmp11_real_sdcf_48b_wfirst") or "slmp11_real_sdcf_48b_wfirst"
        if cfg_prof not in ("auto", "slmp11_real_sdcf_auto"):
            stats.active_resp_profile = cfg_prof
            stats.locked_resp_profile = cfg_prof
            return cfg_prof, ""

        if stats.locked_resp_profile:
            stats.active_resp_profile = stats.locked_resp_profile
            return stats.locked_resp_profile, ""

        if cfg_prof == "slmp11_real_sdcf_auto":
            pair = ("slmp11_real_sdcf_48b_wfirst", "slmp11_real_sdcf_48b")
            csr_on = bool((ry_bytes[7] >> 7) & 0x01)
            user_ry_active = any(b != 0 for b in ry_bytes[:7]) or ((ry_bytes[7] & 0x7F) != 0)
            prof = pair[stats.sdcf_body_order_idx]
            stats.active_resp_profile = prof

            if csr_on:
                if stats.csr_high_since == 0.0:
                    stats.csr_high_since = now
                    return prof, (
                        f"[Cyclic-61450 CSR=1] 收到 AX5S CSR 啟動，回應 SSR 採用格式: {prof}"
                    )
                held_s = now - stats.csr_high_since
                if user_ry_active or held_s >= 6.5:
                    stats.locked_resp_profile = prof
                    return prof, (
                        f"[Cyclic-61450 LOCKED] CSR/SSR 握手穩定 ({held_s:.1f}s)! "
                        f"永久鎖定格式: {prof}"
                    )
                return prof, ""

            if stats.csr_high_since > 0.0:
                held_s = now - stats.csr_high_since
                stats.csr_high_since = 0.0
                if 2.0 <= held_s <= 7.0:
                    stats.sdcf_body_order_idx = 1 - stats.sdcf_body_order_idx
                    stats.csr_switch_count += 1
                    new_prof = pair[stats.sdcf_body_order_idx]
                    stats.active_resp_profile = new_prof
                    return new_prof, (
                        f"[Cyclic-61450 AUTO-SWITCH] CSR 維持 {held_s:.1f}s 後降回 0，"
                        f"自動切換 RX/RWr 順序 -> {new_prof}"
                    )
            return prof, ""

        has_active_ry = axis.csr_active or any(b != 0 for b in ry_bytes)

        if has_active_ry:
            prof = stats.active_resp_profile or SLMP_0E70_PROBE_ORDER[0]
            stats.locked_resp_profile = prof
            stats.active_resp_profile = prof
            return prof, (
                f"[Cyclic-61450 LOCKED] AX5S 已連線並送出 RY! 鎖定回應格式: {prof} "
                f"(RY={ry_bytes.hex(' ').upper()})"
            )

        if (
            stats.active_resp_profile
            and stats.rx_packets >= 3
            and 0.5 < scan_ms < 250.0
            and stats.avg_scan_ms < 320.0
        ):
            stats.fast_rate_hold_hits += 1
            if stats.fast_rate_hold_hits <= 15:
                msg = ""
                if stats.fast_rate_hold_hits == 1:
                    msg = (
                        f"[Cyclic-61450 PROBE] 偵測到連續掃描週期加速 ({scan_ms:.1f}ms < 250ms)，"
                        f"暫時維持格式等待 RY 握手: {stats.active_resp_profile}"
                    )
                return stats.active_resp_profile, msg
        else:
            stats.fast_rate_hold_hits = 0

        if not stats.active_resp_profile:
            stats.probe_index = 0
            stats.probe_packet_hits = 1
            prof = SLMP_0E70_PROBE_ORDER[0]
            stats.active_resp_profile = prof
            return prof, f"[Cyclic-61450 PROBE #1/{len(SLMP_0E70_PROBE_ORDER)}] 嘗試格式: {prof}"

        if stats.probe_packet_hits < 2:
            stats.probe_packet_hits += 1
            return stats.active_resp_profile, ""

        stats.probe_index = (stats.probe_index + 1) % len(SLMP_0E70_PROBE_ORDER)
        stats.probe_packet_hits = 1
        prof = SLMP_0E70_PROBE_ORDER[stats.probe_index]
        stats.active_resp_profile = prof
        return prof, (
            f"[Cyclic-61450 PROBE #{stats.probe_index + 1}/{len(SLMP_0E70_PROBE_ORDER)}] "
            f"嘗試格式: {prof}"
        )

    def _process_for_axis(
        self,
        axis: ServoAxis,
        data: bytes,
        addr: Tuple[str, int],
        now: float,
        station_override_idx: Optional[int],
    ) -> None:
        # Cross-socket deduplication: if another socket (e.g. Cyclic vs Broadcast) just processed
        # this exact same packet (including frame_seq + timestamp in data[:64]) within 30 ms, skip.
        sig_head = data[:64]
        prev = self.dedup_cache.get(axis.axis_id)
        if prev is not None:
            prev_tag, prev_addr, prev_head, prev_ts = prev
            if (
                prev_tag != self.source_tag
                and prev_addr == addr
                and prev_head == sig_head
                and (now - prev_ts) < 0.030
            ):
                return
        self.dedup_cache[axis.axis_id] = (self.source_tag, addr, sig_head, now)

        stats = self.stats_by_axis.setdefault(axis.axis_id, CyclicFrameStats())
        parsed = CyclicFrameCodec.extract_ry_rww(
            data, axis, self.cfg, station_override_idx=station_override_idx
        )
        if parsed is None:
            # 若為有效的 0E70h 多站封包且內含 IP 表，代表此軸為保留站 (0.0.0.0) 或不在本封包站台內，靜默略過
            slmp_info = CyclicFrameCodec.detect_slmp_0e70(data)
            if slmp_info is not None and len(data) >= slmp_info[1] + 64:
                return
            stats.malformed_packets += 1
            if self.on_network_diag is not None and (
                stats.malformed_packets <= 5 or stats.malformed_packets % 100 == 0
            ):
                hex_preview = data[:128].hex(" ").upper()
                self.on_network_diag(
                    axis.axis_id,
                    f"[{self.source_tag} ERR #{stats.malformed_packets}] From {addr[0]}:{addr[1]} | "
                    f"len={len(data)} (無法解析 RY/RWw) | HEX={hex_preview}",
                )
            return

        ry_bytes, rww_bytes, hdr = parsed

        # Update timing & sequence diagnostics
        was_link_lost = stats.link_lost or stats.timeout_active
        scan_ms = 0.0
        if stats.last_rx_ts > 0.0 and not was_link_lost:
            scan_ms = (now - stats.last_rx_ts) * 1000.0
            stats.last_scan_ms = scan_ms
            if stats.avg_scan_ms <= 0.0:
                stats.avg_scan_ms = scan_ms
            else:
                stats.avg_scan_ms = (stats.avg_scan_ms * 0.85) + (scan_ms * 0.15)
        stats.last_rx_ts = now
        stats.timeout_active = False
        stats.link_lost = False
        if was_link_lost and self.on_network_diag is not None:
            self.on_network_diag(
                axis.axis_id,
                f"[{self.source_tag}] 循環通訊已恢復 (From {addr[0]}:{addr[1]})",
            )

        if hdr.header_len > 0 and stats.last_seq_no >= 0:
            expected = (stats.last_seq_no + 1) & 0xFFFF
            if hdr.frame_seq != expected and hdr.frame_seq != stats.last_seq_no:
                stats.dropped_seq_count += 1
        stats.last_seq_no = hdr.frame_seq
        stats.last_header_len = hdr.header_len
        stats.last_pkt_len = len(data)
        stats.last_protocol_ver = hdr.protocol_version
        stats.last_is_slmp_0e70 = hdr.is_slmp_0e70
        stats.master_ip = addr[0]
        stats.master_port = addr[1]
        stats.rx_packets += 1

        # Execute RY(n+3)F / RX(n+3)F handshake & unpack RWwn / pack RWrn using active OD mapping (Mode 1 default)
        rx_bytes, rwr_bytes = axis.process_cyclic_input(ry_bytes, rww_bytes)

        probe_msg = ""
        resp_profile = SLMP_0E70_PROBE_ORDER[0]
        if hdr.is_slmp_0e70:
            resp_profile, probe_msg = self._select_slmp_resp_profile(
                stats, axis, ry_bytes, rww_bytes, scan_ms, now
            )

        resp = CyclicFrameCodec.build_response(
            hdr, axis, rx_bytes, rwr_bytes, slmp_profile=resp_profile
        )

        out_transport = self.reply_transport_fn(axis) if self.reply_transport_fn is not None else None
        if out_transport is None:
            out_transport = self.transport
        if out_transport is not None:
            out_transport.sendto(resp, addr)
            stats.tx_packets += 1

        if self.on_network_diag is not None:
            shape = (len(data), hdr.header_len, hdr.is_slmp_0e70)
            prev_shape = stats.logged_shape

            if probe_msg:
                self.on_network_diag(
                    axis.axis_id,
                    f"{probe_msg} | len={len(resp)}B TX_HDR={resp[:72].hex(' ').upper()}",
                )

            prev_pdo = stats.last_pdo_payload
            cur_pdo = (ry_bytes, rww_bytes)
            last_pdo_ts = stats.last_pdo_log_ts

            critical_pdo_chg = prev_pdo is not None and (
                prev_pdo[0] != ry_bytes
                or prev_pdo[1][:4] != rww_bytes[:4]
                or prev_pdo[1][12:20] != rww_bytes[12:20]
            )
            if prev_shape != shape or stats.rx_packets == 1:
                stats.logged_shape = shape
                stats.last_pdo_payload = cur_pdo
                stats.last_pdo_log_ts = now
                fmt_label = f"SLMP-{hdr.slmp_frame_type}-0E70h" if hdr.is_slmp_0e70 else "Raw/Legacy"
                self.on_network_diag(
                    axis.axis_id,
                    f"[{self.source_tag}] From {addr[0]}:{addr[1]} | len={len(data)} fmt={fmt_label} "
                    f"hdr={hdr.header_len}B grp={hdr.group_no} sta={hdr.total_stations} seq={hdr.frame_seq} | "
                    f"RY={ry_bytes.hex(' ').upper()} | RX={rx_bytes.hex(' ').upper()}\n"
                    f"  FULL_RX({len(data)}B): {data.hex(' ').upper()}\n"
                    f"  FIRST_TX({len(resp)}B, {resp_profile}): {resp[:80].hex(' ').upper()}",
                )
            elif prev_pdo != cur_pdo and (critical_pdo_chg or (now - last_pdo_ts) >= 0.02):
                # 僅於實際 PDO (RY / RWw) 發生變化時紀錄，避免定期時間戳記跳動洗版日誌
                stats.last_pdo_payload = cur_pdo
                stats.last_pdo_log_ts = now
                rww_w = " ".join(f"{struct.unpack_from('<H', rww_bytes, k * 2)[0]:04X}" for k in range(4))
                rwr_w = " ".join(f"{struct.unpack_from('<H', rwr_bytes, k * 2)[0]:04X}" for k in range(4))
                m_cmd = axis.od.get_int(0x6060, 0x00)
                cw_val = axis.od.get_int(0x6040, 0x00)
                t_pos = axis.od.get_int(0x607A, 0x00)
                t_vel = axis.od.get_int(0x60FF, 0x00)
                p_vel = axis.od.get_int(0x6081, 0x00)
                p_acc = axis.od.get_int(0x6083, 0x00)
                p_dec = axis.od.get_int(0x6084, 0x00)
                m_act = axis.od.get_int(0x6061, 0x00)
                sw_val = axis.od.get_int(0x6041, 0x00)
                d01_val = axis.od.get_int(0x2D11, 0x00)
                d02_val = axis.od.get_int(0x2D12, 0x00)
                a_pos = axis.od.get_int(0x6064, 0x00)
                a_vel = axis.od.get_int(0x606C, 0x00)
                self.on_network_diag(
                    axis.axis_id,
                    f"[{self.source_tag} PDO_CHG #{stats.rx_packets}] "
                    f"RY={ry_bytes.hex(' ').upper()} RX={rx_bytes.hex(' ').upper()} | "
                    f"RWw[0..3]={rww_w} RWr[0..3]={rwr_w} | "
                    f"CMD(6060={m_cmd} 6040=0x{cw_val:04X} 607A={t_pos} 60FF={t_vel} 6081={p_vel} 6083={p_acc} 6084={p_dec}) -> "
                    f"STS(6061={m_act} 6041=0x{sw_val:04X} 2D11=0x{d01_val:04X} 2D12=0x{d02_val:04X} 6064={a_pos} 606C={a_vel})",
                )

        if self.on_cyclic_packet is not None:
            self.on_cyclic_packet(axis.axis_id, hdr)


class IEFBCyclicServer:
    """單軸非同步 UDP Port 61450 CC-Link IE Field Basic 循環通訊伺服器。"""

    def __init__(
        self,
        axis: ServoAxis,
        cfg: Optional[CyclicFrameConfig] = None,
        port: int = IEFB_CYCLIC_PORT,
        on_cyclic_packet: Optional[Callable[[int, CyclicFrameHeader], None]] = None,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
        dedup_cache: Optional[Dict[int, Tuple[str, Tuple[str, int], bytes, float]]] = None,
    ) -> None:
        self.axis = axis
        self.cfg = cfg if cfg is not None else CyclicFrameConfig()
        self.port = port
        self.on_cyclic_packet = on_cyclic_packet
        self.on_network_diag = on_network_diag
        self.dedup_cache = dedup_cache if dedup_cache is not None else {}
        self.stats = CyclicFrameStats()
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.bound_ip: str = ""
        self.is_running: bool = False

    def check_watchdog(self, link_timeout_ms: float = 1500.0) -> bool:
        """檢查循環通訊是否中斷（如網路線拔除或 PLC 停止掃描），逾時則重置連線狀態。"""
        if not self.is_running or self.stats.last_rx_ts <= 0.0:
            return False
        elapsed_ms = (time.perf_counter() - self.stats.last_rx_ts) * 1000.0
        timed_out = False
        if elapsed_ms > link_timeout_ms:
            if not self.stats.link_lost:
                self.stats.link_lost = True
                self.stats.last_scan_ms = 0.0
                self.stats.avg_scan_ms = 0.0
                self.axis.on_communication_lost(
                    f"Cyclic Watchdog Timeout ({elapsed_ms:.0f}ms > {link_timeout_ms:.0f}ms)"
                )
                if self.on_network_diag is not None:
                    self.on_network_diag(
                        self.axis.axis_id,
                        f"[Cyclic-{self.port} TIMEOUT] 超過 {link_timeout_ms / 1000.0:.1f}s 未收到 PLC 循環封包，"
                        f"已重置 CSR/SSR 與伺服致能狀態",
                    )
            timed_out = True
        if elapsed_ms > self.cfg.timeout_ms:
            self.stats.timeout_active = True
            timed_out = True
        return timed_out

    async def start(self) -> Tuple[bool, str]:
        self.stats.last_rx_ts = 0.0
        self.stats.last_scan_ms = 0.0
        self.stats.avg_scan_ms = 0.0
        self.stats.timeout_active = False
        self.stats.link_lost = False
        self.stats.reset_probe_state()
        loop = asyncio.get_running_loop()
        bind_targets = [self.axis.ip_address, "0.0.0.0", "127.0.0.1"]
        last_err = ""
        for target_ip in bind_targets:
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.bind((target_ip, self.port))
                axes_map = {self.axis.station_no: self.axis}
                stats_map = {self.axis.axis_id: self.stats}
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: _IEFBCyclicProtocol(
                        axes_by_station=axes_map,
                        cfg=self.cfg,
                        stats_by_axis=stats_map,
                        on_cyclic_packet=self.on_cyclic_packet,
                        on_network_diag=self.on_network_diag,
                        dedup_cache=self.dedup_cache,
                        source_tag=f"Cyclic-{self.port}",
                    ),
                    sock=sock,
                )
                self.transport = transport  # type: ignore[assignment]
                self.bound_ip = target_ip
                self.is_running = True
                return True, f"IEFB Cyclic UDP/{self.port} listening on {target_ip}"
            except OSError as exc:
                if sock is not None:
                    sock.close()
                last_err = f"{target_ip}:{self.port} -> {exc}"
        self.is_running = False
        return False, f"IEFB Cyclic bind failed: {last_err}"

    def stop(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None
        self.is_running = False
        self.bound_ip = ""
        self.stats.rx_packets = 0
        self.stats.tx_packets = 0
        self.stats.last_rx_ts = 0.0
        self.stats.last_scan_ms = 0.0
        self.stats.avg_scan_ms = 0.0
        self.stats.timeout_active = False
        self.stats.link_lost = False
        self.stats.master_ip = ""
        self.stats.master_port = 0
        self.stats.reset_probe_state()
        self.axis.on_communication_stopped()


class _NodeSearchProtocol(asyncio.DatagramProtocol):
    def __init__(self, processor: SLMPFrameProcessor) -> None:
        self.processor = processor
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        resp = self.processor.handle_packet(data, addr)
        if resp is not None and self.transport is not None:
            self.transport.sendto(resp, addr)


class NodeSearchServer:
    """非同步 UDP Port 61451 伺服器，處理 CC-Link IEFB NodeSearch (0E30h) 與 IPAddressSet (0E31h)。"""

    def __init__(
        self,
        axis: ServoAxis,
        port: int = IEFB_NODESEARCH_PORT,
        on_transaction: Optional[Callable[[SLMPTransaction], None]] = None,
        on_ip_change: Optional[Callable[[int, str, str, str], None]] = None,
        on_raw_packet: Optional[Callable[[int, Tuple[str, int], bytes, str], None]] = None,
    ) -> None:
        self.axis = axis
        self.port = port
        self.processor = SLMPFrameProcessor(
            axis=axis,
            on_transaction=on_transaction,
            on_ip_change=on_ip_change,
            on_raw_packet=on_raw_packet,
        )
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.bound_ip: str = ""
        self.is_running: bool = False

    async def start(self) -> Tuple[bool, str]:
        loop = asyncio.get_running_loop()
        bind_targets = [self.axis.ip_address, "0.0.0.0", "127.0.0.1"]
        last_err = ""
        for target_ip in bind_targets:
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.bind((target_ip, self.port))
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: _NodeSearchProtocol(self.processor),
                    sock=sock,
                )
                self.transport = transport  # type: ignore[assignment]
                self.bound_ip = target_ip
                self.is_running = True
                return True, f"NodeSearch UDP/{self.port} listening on {target_ip}"
            except OSError as exc:
                if sock is not None:
                    sock.close()
                last_err = f"{target_ip}:{self.port} -> {exc}"
        self.is_running = False
        return False, f"NodeSearch bind failed: {last_err}"

    def stop(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None
        self.is_running = False


class _IQSSProtocol(asyncio.DatagramProtocol):
    def __init__(
        self,
        axis: ServoAxis,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
    ) -> None:
        self.axis = axis
        self.on_network_diag = on_network_diag
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        if len(data) < 4 or self.transport is None:
            return
        if self.on_network_diag is not None:
            self.on_network_diag(
                self.axis.axis_id,
                f"[iQSS-45237] From {addr[0]}:{addr[1]} len={len(data)} HEX={data[:32].hex(' ').upper()}",
            )
        vendor = VENDOR_NAME.encode("ascii")[:32].ljust(32, b"\x00")
        model = DEVICE_MODEL.encode("ascii")[:16].ljust(16, b"\x00")
        ip_b = ipaddress.IPv4Address(self.axis.ip_address).packed
        mask_b = ipaddress.IPv4Address(self.axis.subnet_mask).packed
        gw_b = ipaddress.IPv4Address(self.axis.gateway).packed
        resp = (
            b"IQSS"
            + struct.pack(
                "<HHHH6s4s4s4s",
                CSP_VENDOR_CODE,
                PRODUCT_ID,
                DEVICE_TYPE_ID,
                0x0001,
                self.axis.mac_address,
                ip_b,
                mask_b,
                gw_b,
            )
            + vendor
            + model
        )
        self.transport.sendto(resp, addr)


class IQSSDiscoveryServer:
    """非同步 UDP Port 45237 iQ Sensor Solution (iQSS) 裝置探索與管理回應伺服器。"""

    def __init__(
        self,
        axis: ServoAxis,
        port: int = IQSS_DISCOVERY_PORT,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
    ) -> None:
        self.axis = axis
        self.port = port
        self.on_network_diag = on_network_diag
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.bound_ip: str = ""
        self.is_running: bool = False

    async def start(self) -> Tuple[bool, str]:
        loop = asyncio.get_running_loop()
        bind_targets = [self.axis.ip_address, "0.0.0.0", "127.0.0.1"]
        last_err = ""
        for target_ip in bind_targets:
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.bind((target_ip, self.port))
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: _IQSSProtocol(self.axis, on_network_diag=self.on_network_diag),
                    sock=sock,
                )
                self.transport = transport  # type: ignore[assignment]
                self.bound_ip = target_ip
                self.is_running = True
                return True, f"iQSS UDP/{self.port} listening on {target_ip}"
            except OSError as exc:
                if sock is not None:
                    sock.close()
                last_err = f"{target_ip}:{self.port} -> {exc}"
        self.is_running = False
        return False, f"iQSS bind failed: {last_err}"

    def stop(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None
        self.is_running = False


class _SharedNodeSearchBroadcastProtocol(asyncio.DatagramProtocol):
    """將 0.0.0.0:61451 收到的 NodeSearch (0E30h) / IPSet (0E31h) 廣播封包分派至各已註冊模擬軸。"""

    def __init__(self, bundles: Dict[int, "AxisNetworkBundle"]) -> None:
        self.bundles = bundles
        self.transport: Optional[asyncio.DatagramTransport] = None
        self._dedup: Dict[int, Tuple[Tuple[str, int], bytes, float]] = {}

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        now = time.perf_counter()
        sig = data[:24]
        for ax_id in sorted(self.bundles.keys()):
            bundle = self.bundles[ax_id]
            prev = self._dedup.get(ax_id)
            if prev is not None and prev[0] == addr and prev[1] == sig and (now - prev[2]) < 0.0005:
                continue
            self._dedup[ax_id] = (addr, sig, now)
            resp = bundle.nodesearch_server.processor.handle_packet(data, addr)
            if resp is not None:
                out_t = bundle.nodesearch_server.transport or self.transport
                if out_t is not None:
                    out_t.sendto(resp, addr)


class AxisNetworkBundle:
    """管理單一模擬軸所需之全部通訊埠（UDP 61450/61451/5010/45237 及 TCP 5010）。"""

    def __init__(
        self,
        axis: ServoAxis,
        cyclic_cfg: Optional[CyclicFrameConfig] = None,
        on_slmp_transaction: Optional[Callable[[SLMPTransaction], None]] = None,
        on_object_modified: Optional[Callable[[int, int, int, str], None]] = None,
        on_ip_change: Optional[Callable[[int, str, str, str], None]] = None,
        on_cyclic_packet: Optional[Callable[[int, CyclicFrameHeader], None]] = None,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
        dedup_cache: Optional[Dict[int, Tuple[str, Tuple[str, int], bytes, float]]] = None,
    ) -> None:
        self.axis = axis
        self.cyclic_cfg = cyclic_cfg if cyclic_cfg is not None else CyclicFrameConfig()
        self.on_network_diag = on_network_diag

        def _on_raw_slmp(ax_id: int, addr: Tuple[str, int], data: bytes, reason: str) -> None:
            if on_network_diag is not None:
                on_network_diag(
                    ax_id,
                    f"[SLMP/NodeSearch Raw] From {addr[0]}:{addr[1]} | {reason} | "
                    f"len={len(data)} HEX={data[:64].hex(' ').upper()}",
                )

        self.cyclic_server = IEFBCyclicServer(
            axis,
            self.cyclic_cfg,
            on_cyclic_packet=on_cyclic_packet,
            on_network_diag=on_network_diag,
            dedup_cache=dedup_cache,
        )
        self.nodesearch_server = NodeSearchServer(
            axis,
            on_transaction=on_slmp_transaction,
            on_ip_change=on_ip_change,
            on_raw_packet=_on_raw_slmp,
        )
        self.slmp_server = SLMPServer(
            axis,
            on_transaction=on_slmp_transaction,
            on_object_modified=on_object_modified,
            on_ip_change=on_ip_change,
            on_raw_packet=_on_raw_slmp,
        )
        self.iqss_server = IQSSDiscoveryServer(axis, on_network_diag=on_network_diag)

    async def start_all(self) -> List[Tuple[str, bool, str]]:
        results = []
        for label, srv in (
            ("Cyclic-61450", self.cyclic_server),
            ("NodeSearch-61451", self.nodesearch_server),
            ("SLMP-5010", self.slmp_server),
            ("iQSS-45237", self.iqss_server),
        ):
            ok, msg = await srv.start()
            results.append((label, ok, msg))
        return results

    def stop_all(self) -> None:
        self.cyclic_server.stop()
        self.nodesearch_server.stop()
        self.slmp_server.stop()
        self.iqss_server.stop()

    @property
    def all_running(self) -> bool:
        return (
            self.cyclic_server.is_running
            and self.nodesearch_server.is_running
            and self.slmp_server.is_running
            and self.iqss_server.is_running
        )


class MultiAxisCyclicManager:
    """
    協調多台模擬伺服軸與其 UDP 61450/61451/5010/45237 及 TCP 5010 伺服器：
    - 支援多軸獨立綁定靜態 IP（例如 192.168.3.1, .2, ...）
    - 自動開啟 0.0.0.0 廣播伴隨接聽器，確保 AX5S 廣播式 0E70h / 0E30h 封包不漏接
    - 透過來源軸 IP Socket 回覆並搭配跨 Socket 30ms 去重，避免重複計數與 ARP 錯位
    """

    def __init__(
        self,
        cyclic_cfg: Optional[CyclicFrameConfig] = None,
        on_network_diag: Optional[Callable[[int, str], None]] = None,
    ) -> None:
        self.cyclic_cfg = cyclic_cfg if cyclic_cfg is not None else CyclicFrameConfig()
        self.on_network_diag = on_network_diag
        self.bundles: Dict[int, AxisNetworkBundle] = {}
        self._dedup_cache: Dict[int, Tuple[str, Tuple[str, int], bytes, float]] = {}
        self._shared_transport: Optional[asyncio.DatagramTransport] = None
        self._broadcast_cyclic_transport: Optional[asyncio.DatagramTransport] = None
        self._broadcast_ns_transport: Optional[asyncio.DatagramTransport] = None
        self._shared_stats: Dict[int, CyclicFrameStats] = {}

    def add_axis(self, axis: ServoAxis, **callbacks) -> AxisNetworkBundle:
        if "on_network_diag" not in callbacks and self.on_network_diag is not None:
            callbacks["on_network_diag"] = self.on_network_diag
        bundle = AxisNetworkBundle(
            axis=axis,
            cyclic_cfg=self.cyclic_cfg,
            dedup_cache=self._dedup_cache,
            **callbacks,
        )
        self.bundles[axis.axis_id] = bundle
        return bundle

    def remove_axis(self, axis_id: int) -> None:
        bundle = self.bundles.pop(axis_id, None)
        if bundle is not None:
            bundle.stop_all()

    async def start_all(self) -> Dict[int, List[Tuple[str, bool, str]]]:
        self.stop_broadcast_listeners()
        report: Dict[int, List[Tuple[str, bool, str]]] = {}
        for ax_id in sorted(self.bundles.keys()):
            report[ax_id] = await self.bundles[ax_id].start_all()
        await self.start_broadcast_listeners()
        return report

    def stop_all(self) -> None:
        self.stop_broadcast_listeners()
        for bundle in self.bundles.values():
            bundle.stop_all()
        if self._shared_transport is not None:
            self._shared_transport.close()
            self._shared_transport = None

    async def start_broadcast_listeners(self) -> List[str]:
        """
        Bind companion 0.0.0.0:61450 & 0.0.0.0:61451 broadcast dispatchers whenever axes are
        bound to specific static IPs so AX5S broadcast 0E70h / 0E30h packets are never dropped.
        """
        self.stop_broadcast_listeners()
        msgs: List[str] = []
        if not self.bundles:
            return msgs

        loop = asyncio.get_running_loop()

        # 1) Broadcast listener on UDP 61450 (only if no bundle already bound 0.0.0.0:61450)
        cyclic_already_any = any(b.cyclic_server.bound_ip == "0.0.0.0" for b in self.bundles.values())
        if not cyclic_already_any:
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.bind(("0.0.0.0", IEFB_CYCLIC_PORT))
                axes_by_sta = {b.axis.station_no: b.axis for b in self.bundles.values()}
                stats_by_ax = {b.axis.axis_id: b.cyclic_server.stats for b in self.bundles.values()}
                bundles_by_ax = dict(self.bundles)

                def _reply_via_axis(ax: ServoAxis) -> Optional[asyncio.DatagramTransport]:
                    b = bundles_by_ax.get(ax.axis_id)
                    return b.cyclic_server.transport if b is not None else None

                transport, _ = await loop.create_datagram_endpoint(
                    lambda: _IEFBCyclicProtocol(
                        axes_by_station=axes_by_sta,
                        cfg=self.cyclic_cfg,
                        stats_by_axis=stats_by_ax,
                        on_network_diag=self.on_network_diag,
                        reply_transport_fn=_reply_via_axis,
                        dedup_cache=self._dedup_cache,
                        source_tag="Broadcast-61450",
                    ),
                    sock=sock,
                )
                self._broadcast_cyclic_transport = transport  # type: ignore[assignment]
                msgs.append(f"Broadcast listener UDP/{IEFB_CYCLIC_PORT} active on 0.0.0.0")
            except OSError as exc:
                if sock is not None:
                    sock.close()
                msgs.append(f"Broadcast listener UDP/{IEFB_CYCLIC_PORT} skipped ({exc})")

        # 2) Broadcast listener on UDP 61451 (NodeSearch 0E30h)
        ns_already_any = any(b.nodesearch_server.bound_ip == "0.0.0.0" for b in self.bundles.values())
        if not ns_already_any:
            sock_ns: Optional[socket.socket] = None
            try:
                sock_ns = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock_ns.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock_ns.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock_ns.bind(("0.0.0.0", IEFB_NODESEARCH_PORT))
                transport_ns, _ = await loop.create_datagram_endpoint(
                    lambda: _SharedNodeSearchBroadcastProtocol(self.bundles),
                    sock=sock_ns,
                )
                self._broadcast_ns_transport = transport_ns  # type: ignore[assignment]
                msgs.append(f"Broadcast listener UDP/{IEFB_NODESEARCH_PORT} active on 0.0.0.0")
            except OSError as exc:
                if sock_ns is not None:
                    sock_ns.close()
                msgs.append(f"Broadcast listener UDP/{IEFB_NODESEARCH_PORT} skipped ({exc})")

        return msgs

    def stop_broadcast_listeners(self) -> None:
        if self._broadcast_cyclic_transport is not None:
            self._broadcast_cyclic_transport.close()
            self._broadcast_cyclic_transport = None
        if self._broadcast_ns_transport is not None:
            self._broadcast_ns_transport.close()
            self._broadcast_ns_transport = None

    async def start_shared_multiplexer(self, bind_ip: str = "0.0.0.0", port: int = IEFB_CYCLIC_PORT) -> Tuple[bool, str]:
        """Start a single UDP 61450 multiplexer routing multi-station packets across all registered axes."""
        loop = asyncio.get_running_loop()
        sock: Optional[socket.socket] = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((bind_ip, port))
            axes_by_sta = {b.axis.station_no: b.axis for b in self.bundles.values()}
            transport, _ = await loop.create_datagram_endpoint(
                lambda: _IEFBCyclicProtocol(
                    axes_by_station=axes_by_sta,
                    cfg=self.cyclic_cfg,
                    stats_by_axis=self._shared_stats,
                    on_network_diag=self.on_network_diag,
                    dedup_cache=self._dedup_cache,
                    source_tag=f"Mux-{port}",
                ),
                sock=sock,
            )
            self._shared_transport = transport  # type: ignore[assignment]
            return True, f"Shared IEFB Multiplexer UDP/{port} listening on {bind_ip}"
        except OSError as exc:
            if sock is not None:
                sock.close()
            return False, f"Shared Multiplexer bind failed: {exc}"


def build_mode1_rww_packet(
    mode: int = 1,
    controlword: int = 0x0000,
    target_pos: int = 0,
    target_vel: int = 0,
    vel_limit: int = 300000,
    target_torque: int = 0,
    profile_vel: int = 100000,
    profile_accel: int = 200,
    profile_decel: int = 200,
    torque_slope: int = 1000,
    pos_option: int = 0,
    touch_probe_func: int = 0,
) -> bytes:
    """Pack a 64-byte Mode 1 (PA53 = 0) RWwn00~RWwn1F buffer for Master -> Slave transmission."""
    buf = bytearray(64)
    struct.pack_into("<h", buf, 0, mode)                   # RWwn00: 6060h (SINT padded to 16-bit word)
    struct.pack_into("<H", buf, 2, controlword & 0xFFFF)   # RWwn01: 6040h
    # RWwn02~05 (Bytes 4..11): Reserved (0x0000)
    struct.pack_into("<i", buf, 12, target_pos)            # RWwn06~07: 607Ah
    struct.pack_into("<i", buf, 16, target_vel)            # RWwn08~09: 60FFh
    struct.pack_into("<I", buf, 20, vel_limit & 0xFFFFFFFF)  # RWwn0A~0B: 2D20h
    struct.pack_into("<h", buf, 24, target_torque)         # RWwn0C: 6071h
    # RWwn0D (Bytes 26..27): Reserved
    struct.pack_into("<I", buf, 28, profile_vel & 0xFFFFFFFF)    # RWwn0E~0F: 6081h
    struct.pack_into("<I", buf, 32, profile_accel & 0xFFFFFFFF)  # RWwn10~11: 6083h
    struct.pack_into("<I", buf, 36, profile_decel & 0xFFFFFFFF)  # RWwn12~13: 6084h
    struct.pack_into("<I", buf, 40, torque_slope & 0xFFFFFFFF)   # RWwn14~15: 6087h
    struct.pack_into("<H", buf, 44, pos_option & 0xFFFF)         # RWwn16: 60F2h
    struct.pack_into("<H", buf, 46, touch_probe_func & 0xFFFF)   # RWwn17: 60B8h
    return bytes(buf)


def unpack_mode1_rwr_packet(rwr_bytes: bytes) -> Dict[str, int]:
    """Unpack a 64-byte Mode 1 (PA53 = 0) RWrn00~RWrn1F buffer from Slave -> Master."""
    if len(rwr_bytes) < 64:
        rwr_bytes = rwr_bytes.ljust(64, b"\x00")
    return {
        "mode_display_6061": struct.unpack_from("<b", rwr_bytes, 0)[0],
        "statusword_6041": struct.unpack_from("<H", rwr_bytes, 2)[0],
        "sd01_2d11": struct.unpack_from("<H", rwr_bytes, 4)[0],
        "sd02_2d12": struct.unpack_from("<H", rwr_bytes, 6)[0],
        "pos_actual_6064": struct.unpack_from("<i", rwr_bytes, 12)[0],
        "vel_actual_606c": struct.unpack_from("<i", rwr_bytes, 16)[0],
        "following_error_60f4": struct.unpack_from("<i", rwr_bytes, 20)[0],
        "torque_actual_6077": struct.unpack_from("<h", rwr_bytes, 24)[0],
        "error_code_603f": struct.unpack_from("<H", rwr_bytes, 28)[0],
        "tp_status_60b9": struct.unpack_from("<H", rwr_bytes, 32)[0],
        "tp1_pos_60ba": struct.unpack_from("<i", rwr_bytes, 36)[0],
        "tp1_neg_60bb": struct.unpack_from("<i", rwr_bytes, 40)[0],
        "tp2_pos_60bc": struct.unpack_from("<i", rwr_bytes, 44)[0],
        "tp2_neg_60bd": struct.unpack_from("<i", rwr_bytes, 48)[0],
    }


def diagnose_local_network(
    axes: List[ServoAxis],
    master_ip: str = "192.168.3.250",
) -> List[str]:
    """檢查本機 IPv4 網卡設定與 OS 預設送往 PLC 主站的出站路由來源 IP，並提供常見衝突提示。"""
    msgs: List[str] = []
    local_ips: List[str] = []
    try:
        infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        local_ips = sorted({str(info[4][0]) for info in infos if info[4]})
    except OSError as exc:
        msgs.append(f"[NetDiag] 無法列舉本機 IPv4 清單: {exc}")

    if local_ips:
        msgs.append(f"[NetDiag] 本機 IPv4 位址: {', '.join(local_ips)}")

    route_src = ""
    try:
        probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe_sock.connect((master_ip, IEFB_CYCLIC_PORT))
            route_src = str(probe_sock.getsockname()[0])
        finally:
            probe_sock.close()
        msgs.append(f"[NetDiag] Windows 送往 {master_ip}:{IEFB_CYCLIC_PORT} 預設路由來源 IP = {route_src}")
    except OSError as exc:
        msgs.append(f"[NetDiag] 送往 {master_ip} 路由檢查失敗: {exc}")

    for ax in axes:
        if ax.ip_address.endswith(".1"):
            msgs.append(
                f"[NetDiag 提示 Axis {ax.axis_id}] IP {ax.ip_address} 為常見閘道器/路由器尾碼 (.1)，"
                "若網段有交換器/路由器或其他網卡同佔用 .1，PLC ARP 將指向錯誤 MAC 導致 CFE8 無回應；"
                "建議可改測 192.168.3.11。"
            )
        if local_ips and ax.ip_address not in local_ips and route_src != ax.ip_address:
            msgs.append(
                f"[NetDiag 警告 Axis {ax.axis_id}] 本機清單未見 {ax.ip_address}，"
                "請確認已於對接 PLC 的有線網卡綁定此 IP。"
            )
        elif route_src and route_src != ax.ip_address:
            msgs.append(
                f"[NetDiag 提示 Axis {ax.axis_id}] 預設對 PLC 出站 IP ({route_src}) 與軸 IP ({ax.ip_address}) 不同，"
                "請確認無第二張網卡 (Wi-Fi/VPN) 也在 192.168.3.x 網段搶走回傳封包。"
            )
    return msgs


async def probe_real_servo_0e70(
    target_slave_ip: str = "192.168.3.1",
    master_ip: str = "192.168.3.250",
    port: int = IEFB_CYCLIC_PORT,
    timeout_s: float = 2.0,
    send_count: int = 3,
    client_bind_port: Optional[int] = None,
) -> Tuple[bool, Optional[bytes], List[str]]:
    """
    Temporarily act as a CC-Link IEFB Master (AX5S) and send exact 143B SLMP 3E 0E70h
    cyclic polls to a physical SDC-F servo drive to capture its real UDP 61450 response bytes.
    Returns (success, first_reply_bytes, diagnostic_log_lines).
    """
    logs: List[str] = [
        f"=== 開始實機伺服 0E70h 回包探測 (Target={target_slave_ip}:{port}, Master={master_ip}) ==="
    ]

    route_src = ""
    try:
        probe_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe_sock.connect((target_slave_ip, port))
            route_src = str(probe_sock.getsockname()[0])
        finally:
            probe_sock.close()
    except OSError:
        route_src = ""

    if route_src == target_slave_ip and not target_slave_ip.startswith("127."):
        logs.append(
            f"[Probe 警告] 本機目前對 {target_slave_ip} 的出站來源 IP 正好也是 {route_src}！"
            "直連實機伺服前，請將電腦有線網卡 IP 改為 192.168.3.250 (或非 .1 位址)，避免與實機伺服 IP 衝突。"
        )

    bind_port = port if client_bind_port is None else client_bind_port
    sock: Optional[socket.socket] = None
    bound_ip = ""
    effective_master_ip = master_ip

    candidate_binds: List[Tuple[str, str]] = [
        (master_ip, master_ip),
        ("0.0.0.0", route_src or master_ip),
    ]
    if target_slave_ip.startswith("127."):
        candidate_binds.insert(0, ("127.0.0.1", "127.0.0.1"))

    last_err: Optional[OSError] = None
    for b_ip, eff_mip in candidate_binds:
        try:
            cand = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            cand.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            cand.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            cand.bind((b_ip, bind_port))
            cand.setblocking(False)
            sock = cand
            bound_ip = b_ip
            effective_master_ip = eff_mip
            break
        except OSError as exc:
            last_err = exc
            try:
                cand.close()
            except OSError:
                pass

    if sock is None:
        logs.append(
            f"[Probe 失敗] 無法綁定 UDP/{bind_port} ({last_err})。"
            "請先點擊「停止通訊」釋放 UDP 61450，再重新執行抓取。"
        )
        return False, None, logs

    logs.append(
        f"[Probe] UDP Socket 已綁定 {bound_ip}:{sock.getsockname()[1]} (封包內 MasterIP={effective_master_ip})"
    )

    try:
        loop = asyncio.get_running_loop()
        replies: List[Tuple[Tuple[str, int], bytes]] = []
        recv_evt = asyncio.Event()
        capturing = True

        class _ProbeProto(asyncio.DatagramProtocol):
            def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
                if not capturing:
                    return
                sub = struct.unpack_from("<H", data, 0)[0] if len(data) >= 2 else 0
                if sub in (0x0050, 0x0054) and not target_slave_ip.startswith("127."):
                    return  # Skip own broadcast echo
                replies.append((addr, bytes(data)))
                recv_evt.set()

        transport, _ = await loop.create_datagram_endpoint(lambda: _ProbeProto(), sock=sock)
        try:
            ry_idle = b"\x00" * STATION_BIT_BYTES
            ry_csr = bytes.fromhex("00 00 00 00 00 00 00 80")
            # Send 3 CSR=0 warmup polls, then sustain CSR=1 cyclic polls at 25ms intervals
            # with advancing master_timestamp and consistent AX5S scan/timeout header (500ms/3).
            warmup_polls = 3
            max_polls = max(60, send_count)
            ssr_seen = False
            post_ssr_remaining = 3
            base_ts = 0x2033FB25
            last_seq = 0
            for seq in range(1, max_polls + 1):
                last_seq = seq
                is_csr = seq > warmup_polls
                ry_to_send = ry_csr if is_csr else ry_idle
                req_pkt = CyclicFrameCodec.build_slmp_0e70_request(
                    ry_list=[ry_to_send],
                    rww_list=[b"\x00" * STATION_WORD_BYTES],
                    frame_seq=seq,
                    protocol_version=CCIEF_PROTOCOL_VERSION,
                    master_ip=effective_master_ip,
                    group_no=1,
                    slave_ips=[target_slave_ip],
                    scan_interval_ms=500,
                    timeout_count=3,
                    master_timestamp=base_ts + seq * 25,
                )
                if seq in (1, warmup_polls + 1):
                    logs.append(
                        f"  PROBE_TX seq={seq} (CSR={int(is_csr)}, {len(req_pkt)}B): {req_pkt.hex(' ').upper()}"
                    )
                prev_len = len(replies)
                recv_evt.clear()
                transport.sendto(req_pkt, (target_slave_ip, port))

                try:
                    await asyncio.wait_for(
                        recv_evt.wait(),
                        timeout=max(0.25, timeout_s / 4.0) if prev_len == 0 else 0.08,
                    )
                except asyncio.TimeoutError:
                    pass

                if len(replies) > prev_len:
                    latest_raw = replies[-1][1]
                    if len(latest_raw) >= 131 and (latest_raw[130] & 0x80 or latest_raw[66] & 0x80):
                        if not ssr_seen:
                            ssr_seen = True
                            logs.append(f"  [Probe SSR=1 觸發] 第 {seq} 封包 (~{seq * 25}ms) 實機回傳 SSR=1！")
                        else:
                            post_ssr_remaining -= 1
                            if post_ssr_remaining <= 0:
                                break

                await asyncio.sleep(0.025)

            # Send 2 clean CSR=0 idle frames before stopping cyclic traffic
            capturing = False
            for tail_idx in range(1, 3):
                tail_seq = last_seq + tail_idx
                idle_tail = CyclicFrameCodec.build_slmp_0e70_request(
                    ry_list=[ry_idle],
                    rww_list=[b"\x00" * STATION_WORD_BYTES],
                    frame_seq=tail_seq,
                    protocol_version=CCIEF_PROTOCOL_VERSION,
                    master_ip=effective_master_ip,
                    group_no=1,
                    slave_ips=[target_slave_ip],
                    scan_interval_ms=500,
                    timeout_count=3,
                    master_timestamp=base_ts + tail_seq * 25,
                )
                transport.sendto(idle_tail, (target_slave_ip, port))
                await asyncio.sleep(0.015)

            if not replies:
                logs.append(
                    f"[Probe 逾時] {timeout_s:.1f} 秒內未收到來自 {target_slave_ip}:{port} 的 UDP 回應。"
                )
                logs.append(
                    "  請確認：(1) 電腦網路線直連實機 SDC-F 伺服 (2) 電腦網卡已設為 192.168.3.250 / 255.255.255.0 "
                    "(3) 實機伺服 IP 為 " + target_slave_ip
                )
                return False, None, logs

            # Deduplicate by payload after seq field (keep first idle, first SSR=1 / change, and latest)
            snapshots: List[Tuple[int, Tuple[str, int], bytes]] = []
            seen_bodies: set[bytes] = set()
            for idx, (peer, raw) in enumerate(replies, start=1):
                body_sig = raw[59:] if len(raw) >= 60 else raw[11:]
                if idx == 1 or idx == len(replies) or body_sig not in seen_bodies:
                    seen_bodies.add(body_sig)
                    snapshots.append((idx, peer, raw))

            first_rx = replies[-1][1]
            for pkt_no, peer, raw in snapshots:
                subhdr = struct.unpack_from("<H", raw, 0)[0] if len(raw) >= 2 else 0
                dlen = struct.unpack_from("<H", raw, 7)[0] if len(raw) >= 9 else 0
                logs.append(
                    f"[Probe 捕獲 #{pkt_no}/{len(replies)}] From {peer[0]}:{peer[1]} | len={len(raw)}B | "
                    f"subhdr=0x{subhdr:04X} slmpDataLen={dlen} (len-9={len(raw)-9})"
                )
                logs.append(f"  REAL_SERVO_0E70_RX({len(raw)}B): {raw.hex(' ').upper()}")
                if len(raw) == 131:
                    rwr_w = " ".join(f"{struct.unpack_from('<H', raw, 59 + k * 2)[0]:04X}" for k in range(16))
                    logs.append(
                        f"    [wfirst 解析] RWr[0..15]={rwr_w} | RX={raw[123:131].hex(' ').upper()}"
                    )
                    logs.append(
                        f"    [舊式 解析]   RX={raw[59:67].hex(' ').upper()} | RWr[0..3]={raw[67:75].hex(' ').upper()}"
                    )
            logs.append(
                "  [提示] 探測停止發送 0E70h 後約 1 秒，實機 SDC-F 因主站通訊中斷監控 (PC.41) 跳 AL.4A 屬正常現象；"
                "重新接回 AX5S 或斷電重送即可清除。"
            )
            return True, first_rx, logs
        finally:
            transport.close()
    except OSError as exc:
        try:
            sock.close()
        except OSError:
            pass
        logs.append(f"[Probe 例外] UDP 送收錯誤: {exc!r}")
        return False, None, logs


def generate_windows_netsh_script(
    axes: List[ServoAxis],
    interface_name: str = "乙太網路",
) -> str:
    """
    產生 Windows 系統管理員批次腳本 (.bat)，透過 netsh 執行：
    1. 為指定網卡批次綁定所有模擬軸的靜態 IP（192.168.3.1, .2, ...）
    2. 新增 Windows 防火牆進出站規則，放行 UDP 61450, 61451, 5010, 45237 通訊埠
    """
    lines = [
        "@echo off",
        "chcp 65001 > nul",
        "echo ======================================================================",
        "echo  Shihlin SDC-F Multi-Axis Simulator - Windows IP & Firewall Setup",
        "echo  Please run this script as Administrator!",
        "echo ======================================================================",
        "",
        f'set IFACE="{interface_name}"',
        "",
        "echo [1/2] Binding Static IP Addresses to %IFACE%...",
    ]
    for ax in axes:
        lines.append(
            f'netsh interface ipv4 add address name=%IFACE% address={ax.ip_address} mask={ax.subnet_mask}'
        )
    lines.extend(
        [
            "",
            "echo [2/2] Adding Windows Firewall In/Out UDP Rules (61450, 61451, 5010, 45237)...",
            'netsh advfirewall firewall delete rule name="SDC-F_CCLink_IEFB_UDP" > nul 2>&1',
            'netsh advfirewall firewall delete rule name="SDC-F_CCLink_IEFB_UDP_OUT" > nul 2>&1',
            'netsh advfirewall firewall add rule name="SDC-F_CCLink_IEFB_UDP" dir=in action=allow protocol=UDP localport=61450,61451,5010,45237 enable=yes profile=any',
            'netsh advfirewall firewall add rule name="SDC-F_CCLink_IEFB_UDP_OUT" dir=out action=allow protocol=UDP localport=61450,61451,5010,45237 enable=yes profile=any',
            "",
            "echo Done! Current IP configuration:",
            "netsh interface ipv4 show addresses name=%IFACE%",
            "echo Current ARP table:",
            "arp -a",
            "pause",
        ]
    )
    return "\r\n".join(lines)


def generate_windows_restore_script(
    axes: List[ServoAxis],
    interface_name: str = "乙太網路",
    mode: str = "dhcp",
    static_ip: str = "192.168.3.250",
    static_mask: str = "255.255.255.0",
    remove_firewall: bool = True,
) -> str:
    """
    產生 Windows 恢復網路與移除防火牆規則批次腳本 (.bat)：
    - mode="dhcp"           : 恢復為自動取得 IP 與 DNS (DHCP)，清除所有多餘靜態 IP
    - mode="delete_sim_ips" : 僅刪除目前模擬軸所綁定之靜態 IP，保留網卡原有主 IP
    - mode="static"         : 覆蓋重設為單一指定固定 IP (static_ip / static_mask)
    - remove_firewall=True  : 一併刪除模擬器建立之 SDC-F_CCLink_IEFB_UDP* 防火牆規則
    """
    lines = [
        "@echo off",
        "chcp 65001 > nul",
        "echo ======================================================================",
        "echo  Shihlin SDC-F Multi-Axis Simulator - Windows Network Restore Script",
        "echo  Please run this script as Administrator!",
        "echo ======================================================================",
        "",
        f'set IFACE="{interface_name}"',
        "",
    ]

    if mode == "delete_sim_ips":
        lines.append("echo [1/2] Removing Simulator Static IPs from %IFACE%...")
        for ax in axes:
            lines.append(
                f'netsh interface ipv4 delete address name=%IFACE% address={ax.ip_address} > nul 2>&1'
            )
    elif mode == "static":
        ip_str = static_ip.strip() or "192.168.3.250"
        mask_str = static_mask.strip() or "255.255.255.0"
        lines.append(f"echo [1/2] Resetting %IFACE% to Single Static IP ({ip_str})...")
        lines.append(
            f'netsh interface ipv4 set address name=%IFACE% source=static address={ip_str} mask={mask_str}'
        )
    else:
        lines.append("echo [1/2] Restoring %IFACE% IPv4 and DNS to DHCP (Auto)...")
        lines.append("netsh interface ipv4 set address name=%IFACE% source=dhcp")
        lines.append("netsh interface ipv4 set dnsservers name=%IFACE% source=dhcp")

    lines.append("")
    if remove_firewall:
        lines.extend(
            [
                "echo [2/2] Removing Simulator Windows Firewall Rules...",
                'netsh advfirewall firewall delete rule name="SDC-F_CCLink_IEFB_UDP" > nul 2>&1',
                'netsh advfirewall firewall delete rule name="SDC-F_CCLink_IEFB_UDP_OUT" > nul 2>&1',
            ]
        )
    else:
        lines.append("echo [2/2] Skipping Windows Firewall Rule removal.")

    lines.extend(
        [
            "",
            "echo Done! Current IP configuration:",
            "netsh interface ipv4 show addresses name=%IFACE%",
            "pause",
        ]
    )
    return "\r\n".join(lines)

