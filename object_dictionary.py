"""
士林電機 SDC-F 物件字典 (Object Dictionary) 與 PDO 映射模組
===========================================================

依據 CSP+ 描述檔 (0x0370_SDC-F_E_CCIEFBasic__1_en.xml) 與 CiA 402 標準定義：
- 裝置識別與通訊參數 (1000h ~ 1FFFh)
- 士林廠商自訂伺服參數 (PA/PB/PC/PD 對應 2000h~23FFh，以及狀態字組 2D11h/2D12h、Pt 限速 2D20h)
- CiA 402 伺服驅動物件 (603Fh ~ 6502h)
- 三種 PDO 映射模式切換 (PA53 = 0: Mode 1 / 1: Mode 2 / 2: Mode 3 自由映射)
- SLMP 4020h 單筆讀寫 (0001h/0002h) 與 SubID 區塊讀寫 (0005h/0006h) 驗證與編解碼
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Callable, Dict, List, Optional, Tuple


# =============================================================================
# 裝置規格與通訊常數（依據 CSP+ XML：0x0370_SDC-F_E_CCIEFBasic__1_en.xml）
# =============================================================================
VENDOR_NAME = "SHIHLIN ELECTRIC"
CSP_VENDOR_CODE = 0x0370
CIA402_VENDOR_ID = 0x05BC
DEVICE_MODEL = "SDC-F"
PRODUCT_ID = 0x000D
DEVICE_TYPE_ID = 0x0021
DEVICE_TYPE_CIA402 = 0x00020192
CCIEF_PROTOCOL_VERSION = 2
OCCUPIED_STATIONS = 1
ENCODER_PPR = 16777216  # 24-bit encoder default (16,777,216 pulses/rev)
RATED_SPEED_RPM = 3000.0


class DataType(str, Enum):
    """CiA 402 / SLMP 物件字典支援的資料型態列舉與位元組長度／有號範圍定義。"""

    SINT = "SINT"       # 8-bit signed
    USINT = "USINT"     # 8-bit unsigned
    INT = "INT"         # 16-bit signed
    UINT = "UINT"       # 16-bit unsigned
    DINT = "DINT"       # 32-bit signed
    UDINT = "UDINT"     # 32-bit unsigned
    STRING = "STRING"   # ASCII string

    @property
    def byte_size(self) -> int:
        if self in (DataType.SINT, DataType.USINT):
            return 1
        if self in (DataType.INT, DataType.UINT):
            return 2
        if self in (DataType.DINT, DataType.UDINT):
            return 4
        return 0

    @property
    def bit_size(self) -> int:
        return self.byte_size * 8

    @property
    def struct_fmt(self) -> str:
        mapping = {
            DataType.SINT: "<b",
            DataType.USINT: "<B",
            DataType.INT: "<h",
            DataType.UINT: "<H",
            DataType.DINT: "<i",
            DataType.UDINT: "<I",
        }
        return mapping[self]

    @property
    def type_bounds(self) -> Tuple[int, int]:
        bounds = {
            DataType.SINT: (-128, 127),
            DataType.USINT: (0, 255),
            DataType.INT: (-32768, 32767),
            DataType.UINT: (0, 65535),
            DataType.DINT: (-2147483648, 2147483647),
            DataType.UDINT: (0, 4294967295),
        }
        return bounds.get(self, (0, 0))


class AccessType(str, Enum):
    """物件存取權限（唯讀 RO / 可讀寫 RW / 唯寫 WO）。"""

    RO = "RO"
    RW = "RW"
    WO = "WO"


class SLMPError(IntEnum):
    """SLMP 4020h 物件字典存取回應之標準 EndCode 錯誤碼。"""

    OK = 0x0000
    UNSUPPORTED_CMD = 0xC059
    PDO_LOCKED_OR_ORDER = 0xCCC7
    READ_WO_OBJECT = 0xCCC8
    WRITE_RO_OBJECT = 0xCCC9
    NON_EXISTENT_INDEX = 0xCCCA
    SIZE_MISMATCH = 0xCCD0
    SIZE_TOO_LARGE = 0xCCD1
    SIZE_TOO_SMALL = 0xCCD2
    NON_EXISTENT_SUBINDEX = 0xCCD3
    VALUE_OUT_OF_RANGE = 0xCCD4
    VALUE_TOO_HIGH = 0xCCD5
    VALUE_TOO_LOW = 0xCCD6


SLMP_ERROR_MESSAGES: Dict[int, str] = {
    SLMPError.OK: "Normal completion",
    SLMPError.UNSUPPORTED_CMD: "Unsupported command or subcommand",
    SLMPError.PDO_LOCKED_OR_ORDER: "PDO mapping locked (CSR=1) or SubIndex 0 not cleared to 0 first",
    SLMPError.READ_WO_OBJECT: "Attempted to read a Write-Only object",
    SLMPError.WRITE_RO_OBJECT: "Attempted to write a Read-Only object",
    SLMPError.NON_EXISTENT_INDEX: "Object Index does not exist",
    SLMPError.SIZE_MISMATCH: "Number of Data Value does not match payload or object size",
    SLMPError.SIZE_TOO_LARGE: "Number of Data Value exceeds object byte length",
    SLMPError.SIZE_TOO_SMALL: "Number of Data Value is smaller than object byte length",
    SLMPError.NON_EXISTENT_SUBINDEX: "Object SubIndex does not exist",
    SLMPError.VALUE_OUT_OF_RANGE: "Written value out of allowable range",
    SLMPError.VALUE_TOO_HIGH: "Written value exceeds maximum limit",
    SLMPError.VALUE_TOO_LOW: "Written value is below minimum limit",
}


@dataclass
class ODEntry:
    """單一物件字典條目，包含 Index/SubIndex、型態、上下限、單位與 Little-Endian 編解碼方法。"""

    index: int
    subindex: int
    name: str
    data_type: DataType
    access: AccessType
    default_value: int | str
    value: int | str
    min_val: int = 0
    max_val: int = 0
    unit: str = ""
    pdo_mappable: bool = True
    description: str = ""

    def __post_init__(self) -> None:
        if self.data_type != DataType.STRING and self.min_val == 0 and self.max_val == 0:
            self.min_val, self.max_val = self.data_type.type_bounds

    @property
    def byte_size(self) -> int:
        if self.data_type == DataType.STRING:
            return len(str(self.value).encode("ascii", errors="replace"))
        return self.data_type.byte_size

    def encode_le(self) -> bytes:
        if self.data_type == DataType.STRING:
            return str(self.value).encode("ascii", errors="replace")
        return struct.pack(self.data_type.struct_fmt, int(self.value))

    def decode_le(self, raw: bytes) -> int | str:
        if self.data_type == DataType.STRING:
            return raw.decode("ascii", errors="replace").rstrip("\x00")
        return struct.unpack(self.data_type.struct_fmt, raw)[0]


@dataclass
class PDOMappingSlot:
    """PDO 32-Word (64B) 固定掃描區間內的單一物件映射槽位。"""

    word_offset: int
    word_count: int
    index: int
    subindex: int
    name: str
    data_type: DataType
    is_reserved: bool = False

    @property
    def byte_offset(self) -> int:
        return self.word_offset * 2

    @property
    def byte_count(self) -> int:
        return self.word_count * 2


# =============================================================================
# PA53 = 0（Mode 1 - 網路標準模式 1，預設）：固定 32-Word RX/TX PDO 映射表
# =============================================================================
MODE1_RX_PDO_LAYOUT: List[PDOMappingSlot] = [
    PDOMappingSlot(0x00, 1, 0x6060, 0x00, "Modes of operation", DataType.SINT, False),
    PDOMappingSlot(0x01, 1, 0x6040, 0x00, "Controlword", DataType.UINT, False),
    PDOMappingSlot(0x02, 4, 0x0000, 0x00, "Reserved (RWwn02~05)", DataType.UINT, True),
    PDOMappingSlot(0x06, 2, 0x607A, 0x00, "Target position", DataType.DINT, False),
    PDOMappingSlot(0x08, 2, 0x60FF, 0x00, "Target velocity", DataType.DINT, False),
    PDOMappingSlot(0x0A, 2, 0x2D20, 0x00, "Velocity limit value", DataType.UDINT, False),
    PDOMappingSlot(0x0C, 1, 0x6071, 0x00, "Target torque", DataType.INT, False),
    PDOMappingSlot(0x0D, 1, 0x0000, 0x00, "Reserved (RWwn0D)", DataType.UINT, True),
    PDOMappingSlot(0x0E, 2, 0x6081, 0x00, "Profile velocity", DataType.UDINT, False),
    PDOMappingSlot(0x10, 2, 0x6083, 0x00, "Profile acceleration", DataType.UDINT, False),
    PDOMappingSlot(0x12, 2, 0x6084, 0x00, "Profile deceleration", DataType.UDINT, False),
    PDOMappingSlot(0x14, 2, 0x6087, 0x00, "Torque slope", DataType.UDINT, False),
    PDOMappingSlot(0x16, 1, 0x60F2, 0x00, "Positioning option code", DataType.UINT, False),
    PDOMappingSlot(0x17, 1, 0x60B8, 0x00, "Touch probe function", DataType.UINT, False),
    PDOMappingSlot(0x18, 8, 0x0000, 0x00, "Reserved (RWwn18~1F)", DataType.UINT, True),
]

MODE1_TX_PDO_LAYOUT: List[PDOMappingSlot] = [
    PDOMappingSlot(0x00, 1, 0x6061, 0x00, "Modes of operation display", DataType.SINT, False),
    PDOMappingSlot(0x01, 1, 0x6041, 0x00, "Statusword", DataType.UINT, False),
    PDOMappingSlot(0x02, 1, 0x2D11, 0x00, "S D01 (DO/Status 1)", DataType.UINT, False),
    PDOMappingSlot(0x03, 1, 0x2D12, 0x00, "S D02 (DO/Status 2)", DataType.UINT, False),
    PDOMappingSlot(0x04, 2, 0x0000, 0x00, "Reserved (RWrn04~05)", DataType.UINT, True),
    PDOMappingSlot(0x06, 2, 0x6064, 0x00, "Position actual value", DataType.DINT, False),
    PDOMappingSlot(0x08, 2, 0x606C, 0x00, "Velocity actual value", DataType.DINT, False),
    PDOMappingSlot(0x0A, 2, 0x60F4, 0x00, "Following error actual value", DataType.DINT, False),
    PDOMappingSlot(0x0C, 1, 0x6077, 0x00, "Torque actual value", DataType.INT, False),
    PDOMappingSlot(0x0D, 1, 0x0000, 0x00, "Reserved (RWrn0D)", DataType.UINT, True),
    PDOMappingSlot(0x0E, 1, 0x603F, 0x00, "Error code", DataType.UINT, False),
    PDOMappingSlot(0x0F, 1, 0x0000, 0x00, "Reserved (RWrn0F)", DataType.UINT, True),
    PDOMappingSlot(0x10, 1, 0x60B9, 0x00, "Touch probe status", DataType.UINT, False),
    PDOMappingSlot(0x11, 1, 0x0000, 0x00, "Reserved (RWrn11)", DataType.UINT, True),
    PDOMappingSlot(0x12, 2, 0x60BA, 0x00, "Touch probe 1 positive edge", DataType.DINT, False),
    PDOMappingSlot(0x14, 2, 0x60BB, 0x00, "Touch probe 1 negative edge", DataType.DINT, False),
    PDOMappingSlot(0x16, 2, 0x60BC, 0x00, "Touch probe 2 positive edge", DataType.DINT, False),
    PDOMappingSlot(0x18, 2, 0x60BD, 0x00, "Touch probe 2 negative edge", DataType.DINT, False),
    PDOMappingSlot(0x1A, 6, 0x0000, 0x00, "Reserved (RWrn1A~1F)", DataType.UINT, True),
]


# =============================================================================
# PA53 = 1（Mode 2 - 標準定位 + 原點復歸擴充模式）：固定 32-Word RX/TX PDO 映射表
# =============================================================================
MODE2_RX_PDO_LAYOUT: List[PDOMappingSlot] = [
    PDOMappingSlot(0x00, 1, 0x6060, 0x00, "Modes of operation", DataType.SINT, False),
    PDOMappingSlot(0x01, 1, 0x6040, 0x00, "Controlword", DataType.UINT, False),
    PDOMappingSlot(0x02, 1, 0x6098, 0x00, "Homing method", DataType.SINT, False),
    PDOMappingSlot(0x03, 1, 0x0000, 0x00, "Reserved (RWwn03)", DataType.UINT, True),
    PDOMappingSlot(0x04, 2, 0x607C, 0x00, "Home offset", DataType.DINT, False),
    PDOMappingSlot(0x06, 2, 0x607A, 0x00, "Target position", DataType.DINT, False),
    PDOMappingSlot(0x08, 2, 0x60FF, 0x00, "Target velocity", DataType.DINT, False),
    PDOMappingSlot(0x0A, 2, 0x2D20, 0x00, "Velocity limit value", DataType.UDINT, False),
    PDOMappingSlot(0x0C, 1, 0x6071, 0x00, "Target torque", DataType.INT, False),
    PDOMappingSlot(0x0D, 1, 0x0000, 0x00, "Reserved (RWwn0D)", DataType.UINT, True),
    PDOMappingSlot(0x0E, 2, 0x6081, 0x00, "Profile velocity", DataType.UDINT, False),
    PDOMappingSlot(0x10, 2, 0x6083, 0x00, "Profile acceleration", DataType.UDINT, False),
    PDOMappingSlot(0x12, 2, 0x6084, 0x00, "Profile deceleration", DataType.UDINT, False),
    PDOMappingSlot(0x14, 2, 0x6099, 0x01, "Homing speed switch", DataType.UDINT, False),
    PDOMappingSlot(0x16, 2, 0x6099, 0x02, "Homing speed zero", DataType.UDINT, False),
    PDOMappingSlot(0x18, 2, 0x609A, 0x00, "Homing acceleration", DataType.UDINT, False),
    PDOMappingSlot(0x1A, 6, 0x0000, 0x00, "Reserved (RWwn1A~1F)", DataType.UINT, True),
]

MODE2_TX_PDO_LAYOUT: List[PDOMappingSlot] = [
    PDOMappingSlot(0x00, 1, 0x6061, 0x00, "Modes of operation display", DataType.SINT, False),
    PDOMappingSlot(0x01, 1, 0x6041, 0x00, "Statusword", DataType.UINT, False),
    PDOMappingSlot(0x02, 1, 0x2D11, 0x00, "S D01 (DO/Status 1)", DataType.UINT, False),
    PDOMappingSlot(0x03, 1, 0x2D12, 0x00, "S D02 (DO/Status 2)", DataType.UINT, False),
    PDOMappingSlot(0x04, 2, 0x60FD, 0x00, "Digital inputs", DataType.UDINT, False),
    PDOMappingSlot(0x06, 2, 0x6064, 0x00, "Position actual value", DataType.DINT, False),
    PDOMappingSlot(0x08, 2, 0x606C, 0x00, "Velocity actual value", DataType.DINT, False),
    PDOMappingSlot(0x0A, 2, 0x60F4, 0x00, "Following error actual value", DataType.DINT, False),
    PDOMappingSlot(0x0C, 1, 0x6077, 0x00, "Torque actual value", DataType.INT, False),
    PDOMappingSlot(0x0D, 1, 0x0000, 0x00, "Reserved (RWrn0D)", DataType.UINT, True),
    PDOMappingSlot(0x0E, 1, 0x603F, 0x00, "Error code", DataType.UINT, False),
    PDOMappingSlot(0x0F, 17, 0x0000, 0x00, "Reserved (RWrn0F~1F)", DataType.UINT, True),
]


class ObjectDictionary:
    """完整的士林 SDC-F CiA 402 + 廠商擴充物件字典與 PDO 編解碼核心。"""

    def __init__(self, axis_id: int = 1) -> None:
        self.axis_id = axis_id
        self.entries: Dict[Tuple[int, int], ODEntry] = {}
        self.pdo_locked: bool = False  # True when CSR RY(n+3)F == 1
        self.rx_slots: List[PDOMappingSlot] = []
        self.tx_slots: List[PDOMappingSlot] = []
        self.on_entry_changed: Optional[Callable[[int, int, int | str, str], None]] = None

        self._build_dictionary()
        self.apply_mapping_mode(0)  # Default: PA53 = 0 (Mode 1)

    def _add(
        self,
        index: int,
        subindex: int,
        name: str,
        data_type: DataType,
        access: AccessType,
        default_value: int | str,
        min_val: int = 0,
        max_val: int = 0,
        unit: str = "",
        pdo_mappable: bool = True,
        description: str = "",
    ) -> None:
        entry = ODEntry(
            index=index,
            subindex=subindex,
            name=name,
            data_type=data_type,
            access=access,
            default_value=default_value,
            value=default_value,
            min_val=min_val,
            max_val=max_val,
            unit=unit,
            pdo_mappable=pdo_mappable,
            description=description,
        )
        self.entries[(index, subindex)] = entry

    def _build_dictionary(self) -> None:
        # 1000h ~ 1FFFh: CiA 402 Communication & Identity Objects
        self._add(0x1000, 0x00, "Device type", DataType.UDINT, AccessType.RO, DEVICE_TYPE_CIA402, pdo_mappable=False)
        self._add(0x1001, 0x00, "Error register", DataType.USINT, AccessType.RO, 0, pdo_mappable=False)
        self._add(0x1008, 0x00, "Manufacturer device name", DataType.STRING, AccessType.RO, DEVICE_MODEL, pdo_mappable=False)
        self._add(0x100A, 0x00, "Manufacturer software version", DataType.STRING, AccessType.RO, "V1.00", pdo_mappable=False)

        self._add(0x1010, 0x00, "Store parameters sub-count", DataType.USINT, AccessType.RO, 1, pdo_mappable=False)
        self._add(0x1010, 0x01, "Save all parameters", DataType.UDINT, AccessType.RW, 1, 0, 0xFFFFFFFF, pdo_mappable=False)

        self._add(0x1011, 0x00, "Restore parameters sub-count", DataType.USINT, AccessType.RO, 1, pdo_mappable=False)
        self._add(0x1011, 0x01, "Restore default parameters", DataType.UDINT, AccessType.RW, 1, 0, 0xFFFFFFFF, pdo_mappable=False)

        self._add(0x1018, 0x00, "Identity object sub-count", DataType.USINT, AccessType.RO, 4, pdo_mappable=False)
        self._add(0x1018, 0x01, "Vendor ID", DataType.UDINT, AccessType.RO, CIA402_VENDOR_ID, pdo_mappable=False)
        self._add(0x1018, 0x02, "Product code", DataType.UDINT, AccessType.RO, PRODUCT_ID, pdo_mappable=False)
        self._add(0x1018, 0x03, "Revision number", DataType.UDINT, AccessType.RO, 0x00010000, pdo_mappable=False)
        self._add(0x1018, 0x04, "Serial number", DataType.UDINT, AccessType.RO, 0x20250000 + self.axis_id, pdo_mappable=False)

        self._add(0x10F3, 0x00, "Diagnosis history sub-count", DataType.USINT, AccessType.RO, 5, pdo_mappable=False)
        self._add(0x10F3, 0x01, "Max messages", DataType.USINT, AccessType.RO, 16, pdo_mappable=False)
        self._add(0x10F3, 0x02, "Newest message", DataType.USINT, AccessType.RW, 0, 0, 16, pdo_mappable=False)
        self._add(0x10F3, 0x03, "Newest acknowledged message", DataType.USINT, AccessType.RW, 0, 0, 16, pdo_mappable=False)
        self._add(0x10F3, 0x04, "New messages available", DataType.USINT, AccessType.RW, 0, 0, 1, pdo_mappable=False)
        self._add(0x10F3, 0x05, "Diagnosis flags", DataType.UINT, AccessType.RW, 0, 0, 0xFFFF, pdo_mappable=False)

        self._add(0x1600, 0x00, "RxPDO Mapping count", DataType.USINT, AccessType.RW, 0, 0, 32, pdo_mappable=False)
        for sub in range(1, 33):
            self._add(
                0x1600,
                sub,
                f"RxPDO Mapping entry {sub}",
                DataType.UDINT,
                AccessType.RW,
                0x00000000,
                0,
                0xFFFFFFFF,
                pdo_mappable=False,
            )

        self._add(0x1A00, 0x00, "TxPDO Mapping count", DataType.USINT, AccessType.RW, 0, 0, 32, pdo_mappable=False)
        for sub in range(1, 33):
            self._add(
                0x1A00,
                sub,
                f"TxPDO Mapping entry {sub}",
                DataType.UDINT,
                AccessType.RW,
                0x00000000,
                0,
                0xFFFFFFFF,
                pdo_mappable=False,
            )

        # 2000h ~ 2FFFh: Shihlin Specific Objects & PA/PB/PC/PD Servo Parameters
        self._add(0x2001, 0x00, "PA01 Control mode & function", DataType.UINT, AccessType.RW, 0x1030, 0, 0xFFFF, description="PA01")
        self._add(0x2006, 0x00, "PA06 Electronic gear numerator (CMX)", DataType.UDINT, AccessType.RW, 16777216, 1, 0xFFFFFFFF, description="PA06 (2006h/2007h)")
        self._add(0x2007, 0x00, "PA07 Electronic gear denominator (CDV)", DataType.UDINT, AccessType.RW, 10000, 1, 0xFFFFFFFF, description="PA07 (2006h/2007h)")
        self._add(0x200C, 0x00, "PA12 In-position range (INP)", DataType.UINT, AccessType.RW, 100, 0, 65535, "pulse", description="PA12")
        self._add(0x2035, 0x00, "PA53 PDO Mapping Mode", DataType.UINT, AccessType.RW, 0, 0, 2, description="0:Mode1, 1:Mode2, 2:Mode3")
        self._add(0x2036, 0x00, "PA54 Communication Unit Selection", DataType.UINT, AccessType.RW, 0, 0, 1, description="0:0.01rpm/ms, 1:pps/pps^2")

        self._add(0x2101, 0x00, "PB01 Auto-tuning mode", DataType.UINT, AccessType.RW, 0x0003, 0, 0xFFFF, description="PB01")
        self._add(0x2106, 0x00, "PB06 Load inertia ratio", DataType.UINT, AccessType.RW, 70, 0, 30000, "0.1x", description="PB06")
        self._add(0x2107, 0x00, "PB07 Model loop gain", DataType.UINT, AccessType.RW, 400, 10, 20000, "0.1rad/s", description="PB07")
        self._add(0x2108, 0x00, "PB08 Position loop gain", DataType.UINT, AccessType.RW, 370, 10, 20000, "0.1/s", description="PB08")
        self._add(0x2109, 0x00, "PB09 Velocity loop gain", DataType.UINT, AccessType.RW, 823, 20, 65535, "0.1rad/s", description="PB09")

        self._add(0x2201, 0x00, "PC01 Internal accel time", DataType.UINT, AccessType.RW, 200, 0, 20000, "ms", description="PC01")
        self._add(0x2202, 0x00, "PC02 Internal decel time", DataType.UINT, AccessType.RW, 200, 0, 20000, "ms", description="PC02")
        self._add(0x2205, 0x00, "PC05 Following error alarm level", DataType.UDINT, AccessType.RW, 393216, 100, 100000000, "pulse", description="PC05")

        self._add(0x2301, 0x00, "PD01 DI1 polarity/function", DataType.UINT, AccessType.RW, 0x0000, 0, 0xFFFF, description="PD01")
        self._add(0x2302, 0x00, "PD02 DI2 polarity/function", DataType.UINT, AccessType.RW, 0x0000, 0, 0xFFFF, description="PD02")
        self._add(0x2303, 0x00, "PD03 DO1 polarity/function", DataType.UINT, AccessType.RW, 0x0001, 0, 0xFFFF, description="PD03")
        self._add(0x2304, 0x00, "PD04 DO2 polarity/function", DataType.UINT, AccessType.RW, 0x0002, 0, 0xFFFF, description="PD04")

        self._add(0x2D11, 0x00, "S D01 (Drive Status/DO 1)", DataType.UINT, AccessType.RO, 0x0000, 0, 0xFFFF, description="SDC-F/MR-J5-G Status DO 1 (Bit2:S_SA Bit12:S_INP Bit13:S_TLC)")
        self._add(0x2D12, 0x00, "S D02 (Drive Status/DO 2)", DataType.UINT, AccessType.RO, 0x0000, 0, 0xFFFF, description="SDC-F/MR-J5-G Status DO 2 (Bit3:S_ZSP Bit4:S_VLC)")
        self._add(0x2D20, 0x00, "Velocity limit value (Pt mode only)", DataType.UDINT, AccessType.RW, 300000, 0, 600000, "0.01 r/min", description="Mode1 RWwn0A~0B (Pt only)")

        # 6000h ~ 6FFFh: CiA 402 Servo Drive Profile Objects
        self._add(0x603F, 0x00, "Error code", DataType.UINT, AccessType.RO, 0x0000)
        self._add(0x6040, 0x00, "Controlword", DataType.UINT, AccessType.RW, 0x0000, 0, 0xFFFF)
        self._add(0x6041, 0x00, "Statusword", DataType.UINT, AccessType.RO, 0x0040)

        self._add(0x605A, 0x00, "Quick stop option code", DataType.INT, AccessType.RW, 2, 0, 8, pdo_mappable=False)
        self._add(0x605B, 0x00, "Shutdown option code", DataType.INT, AccessType.RW, 0, 0, 1, pdo_mappable=False)
        self._add(0x605C, 0x00, "Disable operation option code", DataType.INT, AccessType.RW, 1, 0, 1, pdo_mappable=False)
        self._add(0x605D, 0x00, "Halt option code", DataType.INT, AccessType.RW, 1, 1, 4, pdo_mappable=False)
        self._add(0x605E, 0x00, "Fault reaction option code", DataType.INT, AccessType.RW, 2, 0, 2, pdo_mappable=False)

        self._add(0x6060, 0x00, "Modes of operation", DataType.SINT, AccessType.RW, 0, -128, 127)
        self._add(0x6061, 0x00, "Modes of operation display", DataType.SINT, AccessType.RO, 0)

        self._add(0x6062, 0x00, "Position demand value", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x6063, 0x00, "Position actual internal value", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x6064, 0x00, "Position actual value", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x6067, 0x00, "Position window", DataType.UDINT, AccessType.RW, 100, 0, 0xFFFFFFFF, "pulse")
        self._add(0x6068, 0x00, "Position window time", DataType.UINT, AccessType.RW, 10, 0, 65535, "ms")

        self._add(0x606B, 0x00, "Velocity demand value", DataType.DINT, AccessType.RO, 0, unit="0.01rpm/pps")
        self._add(0x606C, 0x00, "Velocity actual value", DataType.DINT, AccessType.RO, 0, unit="0.01rpm/pps")
        self._add(0x606D, 0x00, "Velocity window", DataType.UINT, AccessType.RW, 2000, 0, 65535, "0.01 r/min", description="PV Target reached window")
        self._add(0x606E, 0x00, "Velocity window time", DataType.UINT, AccessType.RW, 0, 0, 65535, "ms")
        self._add(0x606F, 0x00, "Velocity threshold", DataType.UINT, AccessType.RW, 2000, 0, 65535, "0.01 r/min", description="Zero speed (ZSP) threshold")
        self._add(0x6070, 0x00, "Velocity threshold time", DataType.UINT, AccessType.RW, 0, 0, 65535, "ms")

        self._add(0x6071, 0x00, "Target torque", DataType.INT, AccessType.RW, 0, -5000, 5000, "0.1%")
        self._add(0x6077, 0x00, "Torque actual value", DataType.INT, AccessType.RO, 0, unit="0.1%")

        self._add(0x607A, 0x00, "Target position", DataType.DINT, AccessType.RW, 0, -2147483648, 2147483647, "pulse")
        self._add(0x607C, 0x00, "Home offset", DataType.DINT, AccessType.RW, 0, -2147483648, 2147483647, "pulse")

        self._add(0x607D, 0x00, "Software position limit sub-count", DataType.USINT, AccessType.RO, 2, pdo_mappable=False)
        self._add(0x607D, 0x01, "Min position limit", DataType.DINT, AccessType.RW, -2147483648, -2147483648, 2147483647, "pulse")
        self._add(0x607D, 0x02, "Max position limit", DataType.DINT, AccessType.RW, 2147483647, -2147483648, 2147483647, "pulse")

        self._add(0x607F, 0x00, "Max profile velocity", DataType.UDINT, AccessType.RW, 600000, 0, 0xFFFFFFFF)
        self._add(0x6080, 0x00, "Max motor speed", DataType.UDINT, AccessType.RW, 600000, 0, 600000, "0.01 r/min")
        self._add(0x6081, 0x00, "Profile velocity", DataType.UDINT, AccessType.RW, 100000, 0, 0xFFFFFFFF)
        self._add(0x6083, 0x00, "Profile acceleration", DataType.UDINT, AccessType.RW, 200, 1, 0xFFFFFFFF)
        self._add(0x6084, 0x00, "Profile deceleration", DataType.UDINT, AccessType.RW, 200, 1, 0xFFFFFFFF)
        self._add(0x6085, 0x00, "Quick stop deceleration", DataType.UDINT, AccessType.RW, 100, 1, 0xFFFFFFFF)
        self._add(0x6086, 0x00, "Motion profile type", DataType.INT, AccessType.RW, 0, 0, 1, pdo_mappable=False)
        self._add(0x6087, 0x00, "Torque slope", DataType.UDINT, AccessType.RW, 1000, 1, 0xFFFFFFFF, "0.1%/s")
        self._add(0x608F, 0x00, "Position encoder resolution sub-count", DataType.USINT, AccessType.RO, 2, pdo_mappable=False)
        self._add(0x608F, 0x01, "Encoder increments (Motor resolution)", DataType.UDINT, AccessType.RW, ENCODER_PPR, 1, 0xFFFFFFFF, "pulse/rev", pdo_mappable=False, description="馬達編碼器解析度 (預設16777216)")
        self._add(0x608F, 0x02, "Motor revolutions", DataType.UDINT, AccessType.RW, 1, 1, 0xFFFFFFFF, "rev", pdo_mappable=False)

        self._add(0x6098, 0x00, "Homing method", DataType.SINT, AccessType.RW, 35, -128, 127)
        self._add(0x6099, 0x00, "Homing speeds sub-count", DataType.USINT, AccessType.RO, 2, pdo_mappable=False)
        self._add(0x6099, 0x01, "Speed during search for switch", DataType.UDINT, AccessType.RW, 50000, 0, 0xFFFFFFFF)
        self._add(0x6099, 0x02, "Speed during search for zero", DataType.UDINT, AccessType.RW, 10000, 0, 0xFFFFFFFF)
        self._add(0x609A, 0x00, "Homing acceleration", DataType.UDINT, AccessType.RW, 200, 1, 0xFFFFFFFF)

        self._add(0x60B0, 0x00, "Position offset", DataType.DINT, AccessType.RW, 0, -2147483648, 2147483647, "pulse")
        self._add(0x60B1, 0x00, "Velocity offset", DataType.DINT, AccessType.RW, 0, -2147483648, 2147483647)

        self._add(0x60B8, 0x00, "Touch probe function", DataType.UINT, AccessType.RW, 0x0000, 0, 0xFFFF)
        self._add(0x60B9, 0x00, "Touch probe status", DataType.UINT, AccessType.RO, 0x0000)
        self._add(0x60BA, 0x00, "Touch probe 1 positive edge", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x60BB, 0x00, "Touch probe 1 negative edge", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x60BC, 0x00, "Touch probe 2 positive edge", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x60BD, 0x00, "Touch probe 2 negative edge", DataType.DINT, AccessType.RO, 0, unit="pulse")

        self._add(0x60C5, 0x00, "Max acceleration", DataType.UDINT, AccessType.RW, 0xFFFFFFFF, 1, 0xFFFFFFFF)
        self._add(0x60C6, 0x00, "Max deceleration", DataType.UDINT, AccessType.RW, 0xFFFFFFFF, 1, 0xFFFFFFFF)

        self._add(0x60E0, 0x00, "Positive torque limit value", DataType.UINT, AccessType.RW, 3000, 0, 5000, "0.1%")
        self._add(0x60E1, 0x00, "Negative torque limit value", DataType.UINT, AccessType.RW, 3000, 0, 5000, "0.1%")

        self._add(0x60F2, 0x00, "Positioning option code", DataType.UINT, AccessType.RW, 0x0000, 0, 0xFFFF)
        self._add(0x60F4, 0x00, "Following error actual value", DataType.DINT, AccessType.RO, 0, unit="pulse")
        self._add(0x60FD, 0x00, "Digital inputs", DataType.UDINT, AccessType.RO, 0x00000000)
        self._add(0x60FF, 0x00, "Target velocity", DataType.DINT, AccessType.RW, 0, -2147483648, 2147483647)
        self._add(0x6502, 0x00, "Supported drive modes", DataType.UDINT, AccessType.RO, 0x0000002D, pdo_mappable=False, description="PP(1)/PV(3)/PT(4)/HM(6)")

    def get(self, index: int, subindex: int = 0) -> int | str:
        return self.entries[(index, subindex)].value

    def get_int(self, index: int, subindex: int = 0) -> int:
        return int(self.entries[(index, subindex)].value)

    def set_internal(self, index: int, subindex: int, value: int | str) -> None:
        self.entries[(index, subindex)].value = value

    def set_pdo_locked(self, locked: bool) -> None:
        self.pdo_locked = locked

    def reset_defaults(self) -> None:
        for entry in self.entries.values():
            entry.value = entry.default_value
        self.apply_mapping_mode(self.get_int(0x2035, 0x00))

    def apply_mapping_mode(self, mode: int) -> None:
        """依據 PA53 (2035h) 設定切換 PDO 映射模式（0: Mode 1, 1: Mode 2, 2: Mode 3 自由映射）。"""
        mode = max(0, min(2, int(mode)))
        self.entries[(0x2035, 0x00)].value = mode

        if mode == 0:
            self.rx_slots = [PDOMappingSlot(**vars(s)) for s in MODE1_RX_PDO_LAYOUT]
            self.tx_slots = [PDOMappingSlot(**vars(s)) for s in MODE1_TX_PDO_LAYOUT]
            self._sync_slots_to_mapping_objects(0x1600, self.rx_slots)
            self._sync_slots_to_mapping_objects(0x1A00, self.tx_slots)
        elif mode == 1:
            self.rx_slots = [PDOMappingSlot(**vars(s)) for s in MODE2_RX_PDO_LAYOUT]
            self.tx_slots = [PDOMappingSlot(**vars(s)) for s in MODE2_TX_PDO_LAYOUT]
            self._sync_slots_to_mapping_objects(0x1600, self.rx_slots)
            self._sync_slots_to_mapping_objects(0x1A00, self.tx_slots)
        else:
            if self.get_int(0x1600, 0x00) == 0:
                self.rx_slots = [PDOMappingSlot(**vars(s)) for s in MODE1_RX_PDO_LAYOUT]
                self._sync_slots_to_mapping_objects(0x1600, self.rx_slots)
            else:
                self.rebuild_custom_pdo_from_od(is_rx=True)

            if self.get_int(0x1A00, 0x00) == 0:
                self.tx_slots = [PDOMappingSlot(**vars(s)) for s in MODE1_TX_PDO_LAYOUT]
                self._sync_slots_to_mapping_objects(0x1A00, self.tx_slots)
            else:
                self.rebuild_custom_pdo_from_od(is_rx=False)

    def _sync_slots_to_mapping_objects(self, map_index: int, slots: List[PDOMappingSlot]) -> None:
        entries_encoded: List[int] = []
        for slot in slots:
            if slot.is_reserved:
                for _ in range(slot.word_count):
                    entries_encoded.append(0x00000010)
            else:
                bit_len = 16 if slot.word_count == 1 else 32
                val = ((slot.index & 0xFFFF) << 16) | ((slot.subindex & 0xFF) << 8) | (bit_len & 0xFF)
                entries_encoded.append(val)

        count = min(32, len(entries_encoded))
        self.entries[(map_index, 0x00)].value = count
        for sub in range(1, 33):
            self.entries[(map_index, sub)].value = entries_encoded[sub - 1] if sub <= count else 0

    def rebuild_custom_pdo_from_od(self, is_rx: bool) -> int:
        map_index = 0x1600 if is_rx else 0x1A00
        count = self.get_int(map_index, 0x00)
        new_slots: List[PDOMappingSlot] = []
        word_cursor = 0

        for sub in range(1, count + 1):
            raw_map = self.get_int(map_index, sub)
            if raw_map == 0:
                if word_cursor >= 32:
                    continue
                new_slots.append(
                    PDOMappingSlot(
                        word_offset=word_cursor,
                        word_count=1,
                        index=0x0000,
                        subindex=0x00,
                        name=f"Reserved (Word {word_cursor:02X})",
                        data_type=DataType.UINT,
                        is_reserved=True,
                    )
                )
                word_cursor += 1
                continue

            idx = (raw_map >> 16) & 0xFFFF
            subidx = (raw_map >> 8) & 0xFF
            bit_len = raw_map & 0xFF

            if bit_len not in (8, 16, 32):
                return SLMPError.VALUE_OUT_OF_RANGE

            word_cnt = 1 if bit_len <= 16 else 2
            if word_cursor + word_cnt > 32:
                if idx == 0x0000:
                    continue
                return SLMPError.VALUE_TOO_HIGH

            if idx == 0x0000:
                new_slots.append(
                    PDOMappingSlot(
                        word_offset=word_cursor,
                        word_count=word_cnt,
                        index=0x0000,
                        subindex=0x00,
                        name=f"Reserved (Word {word_cursor:02X})",
                        data_type=DataType.UINT if word_cnt == 1 else DataType.UDINT,
                        is_reserved=True,
                    )
                )
            else:
                entry = self.entries.get((idx, subidx))
                if entry is None:
                    return SLMPError.NON_EXISTENT_INDEX if not self._check_index_exists(idx) else SLMPError.NON_EXISTENT_SUBINDEX
                if not entry.pdo_mappable:
                    return SLMPError.VALUE_OUT_OF_RANGE
                if is_rx and entry.access == AccessType.RO:
                    return SLMPError.WRITE_RO_OBJECT
                new_slots.append(
                    PDOMappingSlot(
                        word_offset=word_cursor,
                        word_count=word_cnt,
                        index=idx,
                        subindex=subidx,
                        name=entry.name,
                        data_type=entry.data_type,
                        is_reserved=False,
                    )
                )
            word_cursor += word_cnt

        if word_cursor < 32:
            new_slots.append(
                PDOMappingSlot(
                    word_offset=word_cursor,
                    word_count=32 - word_cursor,
                    index=0x0000,
                    subindex=0x00,
                    name=f"Reserved (Word {word_cursor:02X}~1F)",
                    data_type=DataType.UINT,
                    is_reserved=True,
                )
            )

        if is_rx:
            self.rx_slots = new_slots
        else:
            self.tx_slots = new_slots
        return SLMPError.OK

    def unpack_rx_pdo(self, rww_bytes: bytes) -> None:
        """從主站送來的 64B RWwn 封包依目前 rx_slots 解包並寫入對應物件（含上下限鉗制）。"""
        if len(rww_bytes) < 64:
            rww_bytes = rww_bytes.ljust(64, b"\x00")

        for slot in self.rx_slots:
            if slot.is_reserved:
                continue
            entry = self.entries.get((slot.index, slot.subindex))
            if entry is None or entry.access == AccessType.RO:
                continue
            off = slot.byte_offset
            if slot.data_type == DataType.SINT:
                val = struct.unpack_from("<b", rww_bytes, off)[0]
            elif slot.data_type == DataType.USINT:
                val = struct.unpack_from("<B", rww_bytes, off)[0]
            elif slot.data_type == DataType.INT:
                val = struct.unpack_from("<h", rww_bytes, off)[0]
            elif slot.data_type == DataType.UINT:
                val = struct.unpack_from("<H", rww_bytes, off)[0]
            elif slot.data_type == DataType.DINT:
                val = struct.unpack_from("<i", rww_bytes, off)[0]
            elif slot.data_type == DataType.UDINT:
                val = struct.unpack_from("<I", rww_bytes, off)[0]
            else:
                continue

            if val < entry.min_val:
                val = entry.min_val
            elif val > entry.max_val:
                val = entry.max_val
            entry.value = val

    def pack_tx_pdo(self) -> bytes:
        """依目前 tx_slots 將伺服回授狀態打包成 64B RWrn 資料回傳給 PLC 主站。"""
        buf = bytearray(64)
        for slot in self.tx_slots:
            if slot.is_reserved:
                continue
            entry = self.entries.get((slot.index, slot.subindex))
            if entry is None:
                continue
            off = slot.byte_offset
            val = int(entry.value)
            if slot.data_type == DataType.SINT:
                struct.pack_into("<H", buf, off, val & 0x00FF)
            elif slot.data_type == DataType.USINT:
                struct.pack_into("<H", buf, off, val & 0x00FF)
            elif slot.data_type == DataType.INT:
                struct.pack_into("<h", buf, off, val)
            elif slot.data_type == DataType.UINT:
                struct.pack_into("<H", buf, off, val & 0xFFFF)
            elif slot.data_type == DataType.DINT:
                struct.pack_into("<i", buf, off, val)
            elif slot.data_type == DataType.UDINT:
                struct.pack_into("<I", buf, off, val & 0xFFFFFFFF)
        return bytes(buf)

    def get_pdo_word_rows(self, is_rx: bool, raw_bytes: Optional[bytes] = None) -> List[Dict[str, object]]:
        slots = self.rx_slots if is_rx else self.tx_slots
        if raw_bytes is None:
            raw_bytes = self.pack_tx_pdo() if not is_rx else b"\x00" * 64
        if len(raw_bytes) < 64:
            raw_bytes = raw_bytes.ljust(64, b"\x00")

        rows: List[Dict[str, object]] = []
        prefix = "RWwn" if is_rx else "RWrn"
        for slot in slots:
            if slot.is_reserved:
                for w in range(slot.word_count):
                    word_idx = slot.word_offset + w
                    word_val = struct.unpack_from("<H", raw_bytes, word_idx * 2)[0]
                    rows.append(
                        {
                            "word_label": f"{prefix}{word_idx:02X}",
                            "byte_range": f"{word_idx * 2}..{word_idx * 2 + 1}",
                            "object": "—",
                            "data_type": "Reserved",
                            "name": "未映射保留區 (Reserved)",
                            "hex_val": f"0x{word_val:04X}",
                            "dec_val": str(word_val),
                            "is_reserved": True,
                        }
                    )
            else:
                obj_label = f"{slot.index:04X}h:{slot.subindex:02X}"
                if slot.word_count == 1:
                    word_idx = slot.word_offset
                    u16_val = struct.unpack_from("<H", raw_bytes, word_idx * 2)[0]
                    if slot.data_type == DataType.SINT:
                        dec_val = struct.unpack_from("<b", raw_bytes, word_idx * 2)[0]
                    elif slot.data_type == DataType.INT:
                        dec_val = struct.unpack_from("<h", raw_bytes, word_idx * 2)[0]
                    else:
                        dec_val = u16_val
                    rows.append(
                        {
                            "word_label": f"{prefix}{word_idx:02X}",
                            "byte_range": f"{word_idx * 2}..{word_idx * 2 + 1}",
                            "object": obj_label,
                            "data_type": slot.data_type.value,
                            "name": slot.name,
                            "hex_val": f"0x{u16_val:04X}",
                            "dec_val": str(dec_val),
                            "is_reserved": False,
                        }
                    )
                else:
                    w0 = slot.word_offset
                    w1 = slot.word_offset + 1
                    u32_val = struct.unpack_from("<I", raw_bytes, w0 * 2)[0]
                    if slot.data_type == DataType.DINT:
                        dec_val = struct.unpack_from("<i", raw_bytes, w0 * 2)[0]
                    else:
                        dec_val = u32_val
                    rows.append(
                        {
                            "word_label": f"{prefix}{w0:02X}~{w1:02X}",
                            "byte_range": f"{w0 * 2}..{w0 * 2 + 3}",
                            "object": obj_label,
                            "data_type": slot.data_type.value,
                            "name": slot.name,
                            "hex_val": f"0x{u32_val:08X}",
                            "dec_val": str(dec_val),
                            "is_reserved": False,
                        }
                    )
        return rows

    def _check_index_exists(self, index: int) -> bool:
        return any(idx == index for (idx, _) in self.entries.keys())

    def slmp_read_object(self, index: int, subindex: int) -> Tuple[int, bytes]:
        """處理 SLMP 4020h Sub 0001h 單一物件讀取，回傳 (EndCode, Little-Endian 資料)。"""
        if not self._check_index_exists(index):
            return SLMPError.NON_EXISTENT_INDEX, b""
        entry = self.entries.get((index, subindex))
        if entry is None:
            return SLMPError.NON_EXISTENT_SUBINDEX, b""
        if entry.access == AccessType.WO:
            return SLMPError.READ_WO_OBJECT, b""
        return SLMPError.OK, entry.encode_le()

    def slmp_write_object(
        self,
        index: int,
        subindex: int,
        num_data_value: int,
        write_data: bytes,
        source: str = "SLMP",
    ) -> int:
        """處理 SLMP 4020h Sub 0002h 單一物件寫入，包含長度對齊、權限、上下限與 PA53/1600h 連動檢查。"""
        if not self._check_index_exists(index):
            return SLMPError.NON_EXISTENT_INDEX
        entry = self.entries.get((index, subindex))
        if entry is None:
            return SLMPError.NON_EXISTENT_SUBINDEX
        if entry.access == AccessType.RO:
            return SLMPError.WRITE_RO_OBJECT

        expected_size = entry.byte_size
        if entry.data_type != DataType.STRING:
            if num_data_value > expected_size:
                return SLMPError.SIZE_TOO_LARGE
            if num_data_value < expected_size:
                return SLMPError.SIZE_TOO_SMALL
            if (
                len(write_data) == num_data_value + (num_data_value % 2)
                and write_data[num_data_value:] == b"\x00"
            ):
                write_data = write_data[:num_data_value]
            if len(write_data) != num_data_value:
                return SLMPError.SIZE_MISMATCH
        else:
            if len(write_data) != num_data_value:
                return SLMPError.SIZE_MISMATCH

        if index in (0x1600, 0x1A00) and self.pdo_locked:
            return SLMPError.PDO_LOCKED_OR_ORDER

        try:
            new_val = entry.decode_le(write_data)
        except struct.error:
            return SLMPError.SIZE_MISMATCH

        if entry.data_type != DataType.STRING:
            ival = int(new_val)
            if ival > entry.max_val:
                return SLMPError.VALUE_TOO_HIGH
            if ival < entry.min_val:
                return SLMPError.VALUE_TOO_LOW

        if index in (0x1600, 0x1A00) and subindex >= 1:
            raw_map = int(new_val)
            t_idx = (raw_map >> 16) & 0xFFFF
            t_sub = (raw_map >> 8) & 0xFF
            t_bits = raw_map & 0xFF
            if raw_map != 0:
                if t_bits not in (8, 16, 32):
                    return SLMPError.VALUE_OUT_OF_RANGE
                if t_idx != 0x0000:
                    target = self.entries.get((t_idx, t_sub))
                    if target is None:
                        return SLMPError.NON_EXISTENT_INDEX if not self._check_index_exists(t_idx) else SLMPError.NON_EXISTENT_SUBINDEX
                    if not target.pdo_mappable:
                        return SLMPError.VALUE_OUT_OF_RANGE
                    if index == 0x1600 and target.access == AccessType.RO:
                        return SLMPError.WRITE_RO_OBJECT

        old_val = entry.value
        entry.value = new_val

        if index == 0x2035 and subindex == 0x00:
            self.apply_mapping_mode(int(new_val))
        elif index in (0x1600, 0x1A00) and subindex == 0x00 and int(new_val) > 0:
            err = self.rebuild_custom_pdo_from_od(is_rx=(index == 0x1600))
            if err != SLMPError.OK:
                entry.value = old_val
                return err
            self.entries[(0x2035, 0x00)].value = 2
        elif index in (0x1600, 0x1A00) and subindex >= 1:
            cur_cnt = self.get_int(index, 0x00)
            if cur_cnt > 0:
                if subindex > cur_cnt:
                    self.entries[(index, 0x00)].value = subindex
                err = self.rebuild_custom_pdo_from_od(is_rx=(index == 0x1600))
                if err != SLMPError.OK:
                    self.entries[(index, 0x00)].value = cur_cnt
                    entry.value = old_val
                    return err
        elif index == 0x1011 and subindex == 0x01 and int(new_val) == 0x64616F6C:
            self.reset_defaults()

        if self.on_entry_changed is not None:
            self.on_entry_changed(index, subindex, new_val, source)

        return SLMPError.OK

    def slmp_read_block(self, index: int, start_subindex: int, count: int) -> Tuple[int, bytes]:
        """處理 SLMP 4020h Sub 0005h 連續 SubIndex 區塊讀取。"""
        if not self._check_index_exists(index):
            return SLMPError.NON_EXISTENT_INDEX, b""
        if count <= 0:
            return SLMPError.VALUE_TOO_LOW, b""

        chunks: List[bytes] = []
        for offset in range(count):
            sub = start_subindex + offset
            if sub > 0xFF:
                return SLMPError.NON_EXISTENT_SUBINDEX, b""
            err, data = self.slmp_read_object(index, sub)
            if err != SLMPError.OK:
                return err, b""
            chunks.append(data)
        return SLMPError.OK, b"".join(chunks)

    def slmp_write_block(
        self,
        index: int,
        start_subindex: int,
        count: int,
        write_data: bytes,
        source: str = "SLMP",
    ) -> int:
        """處理 SLMP 4020h Sub 0006h 連續 SubIndex 區塊寫入（先全數驗證通過再原子性寫入）。"""
        if not self._check_index_exists(index):
            return SLMPError.NON_EXISTENT_INDEX
        if count <= 0:
            return SLMPError.VALUE_TOO_LOW

        cursor = 0
        parsed_writes: List[Tuple[int, int, bytes]] = []
        for offset in range(count):
            sub = start_subindex + offset
            if sub > 0xFF:
                return SLMPError.NON_EXISTENT_SUBINDEX
            entry = self.entries.get((index, sub))
            if entry is None:
                return SLMPError.NON_EXISTENT_SUBINDEX
            if entry.access == AccessType.RO:
                return SLMPError.WRITE_RO_OBJECT
            sz = entry.byte_size
            if cursor + sz > len(write_data):
                return SLMPError.SIZE_TOO_SMALL
            chunk = write_data[cursor : cursor + sz]
            cursor += sz

            if entry.data_type != DataType.STRING:
                val = entry.decode_le(chunk)
                ival = int(val)
                if ival > entry.max_val:
                    return SLMPError.VALUE_TOO_HIGH
                if ival < entry.min_val:
                    return SLMPError.VALUE_TOO_LOW
            parsed_writes.append((sub, sz, chunk))

        if cursor != len(write_data):
            return SLMPError.SIZE_TOO_LARGE

        for sub, sz, chunk in parsed_writes:
            err = self.slmp_write_object(index, sub, sz, chunk, source=source)
            if err != SLMPError.OK:
                return err

        return SLMPError.OK

    def list_all_entries(self) -> List[ODEntry]:
        return [self.entries[k] for k in sorted(self.entries.keys())]
