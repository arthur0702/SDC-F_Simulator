from __future__ import annotations

import asyncio
import ipaddress
import socket
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from object_dictionary import (
    CSP_VENDOR_CODE,
    DEVICE_MODEL,
    DEVICE_TYPE_ID,
    SLMP_ERROR_MESSAGES,
    SLMPError,
)
from servo_axis import ServoAxis


SLMP_UDP_PORT = 5010
SLMP_CMD_OD_ACCESS = 0x4020
SLMP_SUB_READ_OBJ = 0x0001
SLMP_SUB_WRITE_OBJ = 0x0002
SLMP_SUB_READ_BLOCK = 0x0005
SLMP_SUB_WRITE_BLOCK = 0x0006

SLMP_CMD_NODE_SEARCH = 0x0E30
SLMP_CMD_IP_SET = 0x0E31


@dataclass
class SLMPTransaction:
    """紀錄單筆 SLMP / NodeSearch 封包交易明細，供 GUI 日誌顯示。"""

    timestamp: float
    axis_id: int
    peer_ip: str
    peer_port: int
    frame_type: str
    command: int
    subcommand: int
    index: int
    subindex: int
    data_hex: str
    end_code: int
    summary: str


def build_node_search_response_payload(axis: ServoAxis) -> bytes:
    """組裝 NodeSearch (0E30h) 回應資料，回報站台 MAC、IP、Vendor(0x0370) 與型號 SDC-F。"""
    ip_bytes = ipaddress.IPv4Address(axis.ip_address).packed
    mask_bytes = ipaddress.IPv4Address(axis.subnet_mask).packed
    gw_bytes = ipaddress.IPv4Address(axis.gateway).packed
    model_bytes = DEVICE_MODEL.encode("ascii")[:16].ljust(16, b"\x00")
    status = 0x0001 if axis.csr_active else 0x0000

    return struct.pack(
        "<6s4s4s4sHHHHHH16s",
        axis.mac_address,
        ip_bytes,
        mask_bytes,
        gw_bytes,
        0x0002,
        CSP_VENDOR_CODE,
        DEVICE_TYPE_ID,
        0x0001,
        1,
        status,
        model_bytes,
    )


class SLMPFrameProcessor:
    """解析並執行 SLMP 3E/4E Binary 指令封包（支援 4020h 物件存取與 0E30h/0E31h 搜尋設 IP）。"""

    def __init__(
        self,
        axis: ServoAxis,
        on_transaction: Optional[Callable[[SLMPTransaction], None]] = None,
        on_object_modified: Optional[Callable[[int, int, int, str], None]] = None,
        on_ip_change: Optional[Callable[[int, str, str, str], None]] = None,
        on_raw_packet: Optional[Callable[[int, Tuple[str, int], bytes, str], None]] = None,
    ) -> None:
        self.axis = axis
        self.on_transaction = on_transaction
        self.on_object_modified = on_object_modified
        self.on_ip_change = on_ip_change
        self.on_raw_packet = on_raw_packet

    def handle_packet(self, data: bytes, addr: Tuple[str, int]) -> Optional[bytes]:
        if len(data) < 15:
            if self.on_raw_packet is not None:
                self.on_raw_packet(self.axis.axis_id, addr, data, "Packet < 15 bytes")
            return None

        subhdr = struct.unpack_from("<H", data, 0)[0]
        if subhdr == 0x0050:
            frame_type = "3E"
            header_prefix = data[2:7]
            data_len = struct.unpack_from("<H", data, 7)[0]
            if len(data) < 9 + data_len or data_len < 6:
                if self.on_raw_packet is not None:
                    self.on_raw_packet(
                        self.axis.axis_id,
                        addr,
                        data,
                        f"SLMP 3E length mismatch (pkt={len(data)}, data_len={data_len})",
                    )
                return None
            cmd, subcmd = struct.unpack_from("<HH", data, 11)
            payload = data[15 : 9 + data_len]
            resp_subhdr = 0x00D0
            serial_prefix = b""
        elif subhdr == 0x0054:
            if len(data) < 19:
                if self.on_raw_packet is not None:
                    self.on_raw_packet(self.axis.axis_id, addr, data, "SLMP 4E < 19 bytes")
                return None
            frame_type = "4E"
            serial_prefix = data[2:6]
            header_prefix = data[6:11]
            data_len = struct.unpack_from("<H", data, 11)[0]
            if len(data) < 13 + data_len or data_len < 6:
                if self.on_raw_packet is not None:
                    self.on_raw_packet(
                        self.axis.axis_id,
                        addr,
                        data,
                        f"SLMP 4E length mismatch (pkt={len(data)}, data_len={data_len})",
                    )
                return None
            cmd, subcmd = struct.unpack_from("<HH", data, 15)
            payload = data[19 : 13 + data_len]
            resp_subhdr = 0x00D4
        else:
            if self.on_raw_packet is not None:
                self.on_raw_packet(
                    self.axis.axis_id,
                    addr,
                    data,
                    f"Unknown subheader 0x{subhdr:04X}",
                )
            return None

        end_code, resp_payload, idx, subidx, summary = self._dispatch_command(cmd, subcmd, payload)

        body = struct.pack("<H", end_code) + (resp_payload if end_code == SLMPError.OK else b"")
        resp = (
            struct.pack("<H", resp_subhdr)
            + serial_prefix
            + header_prefix
            + struct.pack("<H", len(body))
            + body
        )

        tx = SLMPTransaction(
            timestamp=time.time(),
            axis_id=self.axis.axis_id,
            peer_ip=addr[0],
            peer_port=addr[1],
            frame_type=frame_type,
            command=cmd,
            subcommand=subcmd,
            index=idx,
            subindex=subidx,
            data_hex=payload.hex(" ").upper(),
            end_code=end_code,
            summary=summary,
        )
        if self.on_transaction is not None:
            self.on_transaction(tx)

        return resp

    def _dispatch_command(
        self, cmd: int, subcmd: int, payload: bytes
    ) -> Tuple[int, bytes, int, int, str]:
        if cmd == SLMP_CMD_OD_ACCESS:
            return self._handle_4020(subcmd, payload)

        if cmd == SLMP_CMD_NODE_SEARCH and subcmd == 0x0000:
            return (
                SLMPError.OK,
                build_node_search_response_payload(self.axis),
                0,
                0,
                "NodeSearch (0E30h)",
            )

        if cmd == SLMP_CMD_IP_SET and subcmd == 0x0000:
            if len(payload) < 18:
                return SLMPError.SIZE_TOO_SMALL, b"", 0, 0, "IPAddressSet size error"
            target_mac = payload[0:6]
            if target_mac != self.axis.mac_address and target_mac != b"\xFF\xFF\xFF\xFF\xFF\xFF":
                return SLMPError.VALUE_OUT_OF_RANGE, b"", 0, 0, "IPAddressSet MAC mismatch"
            new_ip = str(ipaddress.IPv4Address(payload[6:10]))
            new_mask = str(ipaddress.IPv4Address(payload[10:14]))
            new_gw = str(ipaddress.IPv4Address(payload[14:18]))
            self.axis.ip_address = new_ip
            self.axis.subnet_mask = new_mask
            self.axis.gateway = new_gw
            if self.on_ip_change is not None:
                self.on_ip_change(self.axis.axis_id, new_ip, new_mask, new_gw)
            return SLMPError.OK, b"", 0, 0, f"IPAddressSet -> {new_ip}"

        return (
            SLMPError.UNSUPPORTED_CMD,
            b"",
            0,
            0,
            f"Unsupported Cmd=0x{cmd:04X} Sub=0x{subcmd:04X}",
        )

    def _handle_4020(self, subcmd: int, payload: bytes) -> Tuple[int, bytes, int, int, str]:
        """處理 SLMP 4020h 物件字典讀寫與區塊讀寫子指令 (0001h/0002h/0005h/0006h)。"""
        if len(payload) < 6:
            return SLMPError.SIZE_TOO_SMALL, b"", 0, 0, "4020h payload < 6 bytes"

        index, subindex, byte3, num_val = struct.unpack_from("<HBBH", payload, 0)

        if subcmd == SLMP_SUB_READ_OBJ:
            end_code, read_data = self.axis.od.slmp_read_object(index, subindex)
            if end_code != SLMPError.OK:
                return end_code, b"", index, subindex, f"Read {index:04X}h:{subindex:02X} ERR=0x{end_code:04X}"
            # 回應標頭第 4 欄必須填入實際回傳位元組長度 (CLPA SLMP 4020h 規範)
            resp_hdr = struct.pack("<HBBH", index, subindex, 0x00, len(read_data))
            return (
                SLMPError.OK,
                resp_hdr + read_data,
                index,
                subindex,
                f"Read {index:04X}h:{subindex:02X} ({len(read_data)}B) -> {read_data.hex(' ').upper()}",
            )

        if subcmd == SLMP_SUB_WRITE_OBJ:
            write_data = payload[6:]
            end_code = self.axis.od.slmp_write_object(
                index, subindex, num_val, write_data, source="SLMP 4020h"
            )
            if end_code != SLMPError.OK:
                return (
                    end_code,
                    b"",
                    index,
                    subindex,
                    f"Write {index:04X}h:{subindex:02X} <= {write_data.hex(' ').upper()} ERR=0x{end_code:04X}",
                )
            if self.on_object_modified is not None:
                self.on_object_modified(self.axis.axis_id, index, subindex, write_data.hex(" ").upper())
            resp_hdr = struct.pack("<HBBH", index, subindex, 0x00, num_val)
            return (
                SLMPError.OK,
                resp_hdr,
                index,
                subindex,
                f"Write {index:04X}h:{subindex:02X} <= {write_data.hex(' ').upper()}",
            )

        if subcmd == SLMP_SUB_READ_BLOCK:
            count = num_val if num_val > 0 else (byte3 if byte3 > 0 else 1)
            end_code, read_data = self.axis.od.slmp_read_block(index, subindex, count)
            if end_code != SLMPError.OK:
                return end_code, b"", index, subindex, f"BlockRead {index:04X}h:{subindex:02X}x{count} ERR=0x{end_code:04X}"
            resp_hdr = struct.pack("<HBBH", index, subindex, 0x00, count)
            return (
                SLMPError.OK,
                resp_hdr + read_data,
                index,
                subindex,
                f"BlockRead {index:04X}h:{subindex:02X} (x{count}) -> {read_data.hex(' ').upper()}",
            )

        if subcmd == SLMP_SUB_WRITE_BLOCK:
            count = num_val if num_val > 0 else (byte3 if byte3 > 0 else 1)
            write_data = payload[6:]
            end_code = self.axis.od.slmp_write_block(
                index, subindex, count, write_data, source="SLMP 4020h Block"
            )
            if end_code != SLMPError.OK:
                return (
                    end_code,
                    b"",
                    index,
                    subindex,
                    f"BlockWrite {index:04X}h:{subindex:02X}x{count} <= {write_data.hex(' ').upper()} ERR=0x{end_code:04X}",
                )
            if self.on_object_modified is not None:
                for offset in range(count):
                    self.on_object_modified(
                        self.axis.axis_id, index, subindex + offset, write_data.hex(" ").upper()
                    )
            resp_hdr = struct.pack("<HBBH", index, subindex, 0x00, count)
            return (
                SLMPError.OK,
                resp_hdr,
                index,
                subindex,
                f"BlockWrite {index:04X}h:{subindex:02X} (x{count}) <= {write_data.hex(' ').upper()}",
            )

        return (
            SLMPError.UNSUPPORTED_CMD,
            b"",
            index,
            subindex,
            f"Unsupported 4020h Subcommand=0x{subcmd:04X}",
        )


class _SLMPUDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, processor: SLMPFrameProcessor) -> None:
        self.processor = processor
        self.transport: Optional[asyncio.DatagramTransport] = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: Tuple[str, int]) -> None:
        resp = self.processor.handle_packet(data, addr)
        if resp is not None and self.transport is not None:
            self.transport.sendto(resp, addr)


class SLMPServer:
    """單軸非同步 SLMP 伺服器，同時監聽 UDP 與 TCP Port 5010 處理 SDO 物件讀寫。"""

    def __init__(
        self,
        axis: ServoAxis,
        port: int = SLMP_UDP_PORT,
        on_transaction: Optional[Callable[[SLMPTransaction], None]] = None,
        on_object_modified: Optional[Callable[[int, int, int, str], None]] = None,
        on_ip_change: Optional[Callable[[int, str, str, str], None]] = None,
        on_raw_packet: Optional[Callable[[int, Tuple[str, int], bytes, str], None]] = None,
    ) -> None:
        self.axis = axis
        self.port = port
        self.processor = SLMPFrameProcessor(
            axis=axis,
            on_transaction=on_transaction,
            on_object_modified=on_object_modified,
            on_ip_change=on_ip_change,
            on_raw_packet=on_raw_packet,
        )
        self.transport: Optional[asyncio.DatagramTransport] = None
        self.tcp_server: Optional[asyncio.AbstractServer] = None
        self.bound_ip: str = ""
        self.is_running: bool = False

    async def _handle_tcp_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        addr: Tuple[str, int] = (
            (str(peer[0]), int(peer[1])) if isinstance(peer, tuple) and len(peer) >= 2 else ("0.0.0.0", 0)
        )
        try:
            while True:
                hdr2 = await reader.readexactly(2)
                subhdr = struct.unpack_from("<H", hdr2, 0)[0]
                if subhdr == 0x0050:
                    rest_hdr = await reader.readexactly(7)
                    data_len = struct.unpack_from("<H", rest_hdr, 5)[0]
                    if data_len < 6 or data_len > 4096:
                        if self.processor.on_raw_packet is not None:
                            self.processor.on_raw_packet(
                                self.axis.axis_id,
                                addr,
                                hdr2 + rest_hdr,
                                f"TCP SLMP 3E invalid data_len={data_len}",
                            )
                        break
                    body = await reader.readexactly(data_len)
                    pkt = hdr2 + rest_hdr + body
                elif subhdr == 0x0054:
                    rest_hdr = await reader.readexactly(11)
                    data_len = struct.unpack_from("<H", rest_hdr, 9)[0]
                    if data_len < 6 or data_len > 4096:
                        if self.processor.on_raw_packet is not None:
                            self.processor.on_raw_packet(
                                self.axis.axis_id,
                                addr,
                                hdr2 + rest_hdr,
                                f"TCP SLMP 4E invalid data_len={data_len}",
                            )
                        break
                    body = await reader.readexactly(data_len)
                    pkt = hdr2 + rest_hdr + body
                else:
                    if self.processor.on_raw_packet is not None:
                        self.processor.on_raw_packet(
                            self.axis.axis_id,
                            addr,
                            hdr2,
                            f"TCP Unknown subheader 0x{subhdr:04X}",
                        )
                    break

                resp = self.processor.handle_packet(pkt, addr)
                if resp is not None:
                    writer.write(resp)
                    await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            pass
        finally:
            try:
                writer.close()
            except OSError:
                pass

    async def start(self) -> Tuple[bool, str]:
        loop = asyncio.get_running_loop()
        bind_targets = [self.axis.ip_address, "0.0.0.0", "127.0.0.1"]
        last_err = ""
        for target_ip in bind_targets:
            sock: Optional[socket.socket] = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((target_ip, self.port))
                transport, _ = await loop.create_datagram_endpoint(
                    lambda: _SLMPUDPProtocol(self.processor),
                    sock=sock,
                )
                self.transport = transport  # type: ignore[assignment]
                self.bound_ip = target_ip
                self.is_running = True

                tcp_note = ""
                try:
                    self.tcp_server = await asyncio.start_server(
                        self._handle_tcp_client,
                        host=target_ip,
                        port=self.port,
                    )
                    tcp_note = f" + TCP/{self.port}"
                except OSError as tcp_exc:
                    self.tcp_server = None
                    tcp_note = f" (TCP skipped: {tcp_exc})"

                return True, f"SLMP UDP/{self.port}{tcp_note} listening on {target_ip}"
            except OSError as exc:
                if sock is not None:
                    sock.close()
                last_err = f"{target_ip}:{self.port} -> {exc}"
        self.is_running = False
        return False, f"SLMP bind failed: {last_err}"

    def stop(self) -> None:
        if self.tcp_server is not None:
            self.tcp_server.close()
            self.tcp_server = None
        if self.transport is not None:
            self.transport.close()
            self.transport = None
        self.is_running = False


@dataclass
class SLMPClientResponse:
    ok: bool
    end_code: int
    error_message: str
    index: int
    subindex: int
    raw_tx: bytes
    raw_rx: bytes
    data_bytes: bytes
    decoded_value: Optional[int] = None


class SLMPClientTester:
    """Builds and sends real SLMP 3E Binary UDP packets to Port 5010."""

    @staticmethod
    def build_3e_frame(
        command: int,
        subcommand: int,
        payload: bytes,
        network_no: int = 0x00,
        pc_no: int = 0xFF,
        dest_io: int = 0x03FF,
        dest_station: int = 0x00,
        monitoring_timer: int = 0x0010,
    ) -> bytes:
        data_len = 6 + len(payload)
        return (
            struct.pack(
                "<HBBHBHHHH",
                0x0050,
                network_no,
                pc_no,
                dest_io,
                dest_station,
                data_len,
                monitoring_timer,
                command,
                subcommand,
            )
            + payload
        )

    @classmethod
    def build_4020_read(cls, index: int, subindex: int) -> bytes:
        payload = struct.pack("<HBBH", index, subindex, 0x00, 0x0000)
        return cls.build_3e_frame(SLMP_CMD_OD_ACCESS, SLMP_SUB_READ_OBJ, payload)

    @classmethod
    def build_4020_write(cls, index: int, subindex: int, write_data: bytes) -> bytes:
        payload = struct.pack("<HBBH", index, subindex, 0x00, len(write_data)) + write_data
        return cls.build_3e_frame(SLMP_CMD_OD_ACCESS, SLMP_SUB_WRITE_OBJ, payload)

    @classmethod
    def build_4020_read_block(cls, index: int, start_subindex: int, count: int) -> bytes:
        payload = struct.pack("<HBBH", index, start_subindex, 0x00, count)
        return cls.build_3e_frame(SLMP_CMD_OD_ACCESS, SLMP_SUB_READ_BLOCK, payload)

    @classmethod
    def build_4020_write_block(cls, index: int, start_subindex: int, count: int, write_data: bytes) -> bytes:
        payload = struct.pack("<HBBH", index, start_subindex, 0x00, count) + write_data
        return cls.build_3e_frame(SLMP_CMD_OD_ACCESS, SLMP_SUB_WRITE_BLOCK, payload)

    @staticmethod
    def encode_integer_le(value: int, byte_size: int, signed: bool = True) -> bytes:
        if byte_size == 1:
            return struct.pack("<b" if signed else "<B", value)
        if byte_size == 2:
            return struct.pack("<h" if signed else "<H", value)
        if byte_size == 4:
            return struct.pack("<i" if signed else "<I", value)
        raise ValueError(f"Unsupported byte size: {byte_size}")

    @staticmethod
    def decode_response(raw_tx: bytes, raw_rx: bytes, index: int, subindex: int, subcommand: int) -> SLMPClientResponse:
        if len(raw_rx) < 11:
            return SLMPClientResponse(
                ok=False,
                end_code=0xFFFF,
                error_message="Malformed SLMP response (< 11 bytes)",
                index=index,
                subindex=subindex,
                raw_tx=raw_tx,
                raw_rx=raw_rx,
                data_bytes=b"",
            )
        subhdr = struct.unpack_from("<H", raw_rx, 0)[0]
        if subhdr != 0x00D0:
            return SLMPClientResponse(
                ok=False,
                end_code=0xFFFF,
                error_message=f"Unexpected subheader 0x{subhdr:04X}",
                index=index,
                subindex=subindex,
                raw_tx=raw_tx,
                raw_rx=raw_rx,
                data_bytes=b"",
            )

        data_len = struct.unpack_from("<H", raw_rx, 7)[0]
        end_code = struct.unpack_from("<H", raw_rx, 9)[0]
        body_payload = raw_rx[11 : 9 + data_len]
        err_msg = SLMP_ERROR_MESSAGES.get(end_code, f"SLMP Error 0x{end_code:04X}")

        if end_code != SLMPError.OK:
            return SLMPClientResponse(
                ok=False,
                end_code=end_code,
                error_message=err_msg,
                index=index,
                subindex=subindex,
                raw_tx=raw_tx,
                raw_rx=raw_rx,
                data_bytes=b"",
            )

        data_bytes = body_payload[6:] if len(body_payload) > 6 else b""
        decoded_val: Optional[int] = None
        if subcommand == SLMP_SUB_READ_OBJ and len(data_bytes) in (1, 2, 4):
            if len(data_bytes) == 1:
                decoded_val = struct.unpack("<B", data_bytes)[0]
            elif len(data_bytes) == 2:
                decoded_val = struct.unpack("<H", data_bytes)[0]
            elif len(data_bytes) == 4:
                decoded_val = struct.unpack("<I", data_bytes)[0]

        return SLMPClientResponse(
            ok=True,
            end_code=end_code,
            error_message=err_msg,
            index=index,
            subindex=subindex,
            raw_tx=raw_tx,
            raw_rx=raw_rx,
            data_bytes=data_bytes,
            decoded_value=decoded_val,
        )

    @classmethod
    def send_request_sync(
        cls,
        target_ip: str,
        subcommand: int,
        index: int,
        subindex: int,
        write_data: bytes = b"",
        block_count: int = 1,
        port: int = SLMP_UDP_PORT,
        timeout_s: float = 1.0,
    ) -> SLMPClientResponse:
        if subcommand == SLMP_SUB_READ_OBJ:
            frame = cls.build_4020_read(index, subindex)
        elif subcommand == SLMP_SUB_WRITE_OBJ:
            frame = cls.build_4020_write(index, subindex, write_data)
        elif subcommand == SLMP_SUB_READ_BLOCK:
            frame = cls.build_4020_read_block(index, subindex, block_count)
        elif subcommand == SLMP_SUB_WRITE_BLOCK:
            frame = cls.build_4020_write_block(index, subindex, block_count, write_data)
        else:
            raise ValueError(f"Unsupported 4020h subcommand: 0x{subcommand:04X}")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(timeout_s)
        try:
            sock.sendto(frame, (target_ip, port))
            rx_data, _ = sock.recvfrom(2048)
            return cls.decode_response(frame, rx_data, index, subindex, subcommand)
        except OSError as exc:
            return SLMPClientResponse(
                ok=False,
                end_code=0xFFFF,
                error_message=f"UDP Error ({target_ip}:{port}): {exc}",
                index=index,
                subindex=subindex,
                raw_tx=frame,
                raw_rx=b"",
                data_bytes=b"",
            )
        finally:
            sock.close()

    @classmethod
    async def send_request_async(
        cls,
        target_ip: str,
        subcommand: int,
        index: int,
        subindex: int,
        write_data: bytes = b"",
        block_count: int = 1,
        port: int = SLMP_UDP_PORT,
        timeout_s: float = 1.0,
    ) -> SLMPClientResponse:
        return await asyncio.to_thread(
            cls.send_request_sync,
            target_ip,
            subcommand,
            index,
            subindex,
            write_data,
            block_count,
            port,
            timeout_s,
        )
