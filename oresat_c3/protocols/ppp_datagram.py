"""Class for the datagrams from the banc3 sband sdr, plus the opcode enum."""
import struct
from enum import IntEnum


class ppp_opcode(IntEnum):
    READ = 0x01
    WRITE = 0x02
    HEARTBEAT = 0x10
    TPDO = 0x11
    RPDO = 0x12
    READ_RSP = 0x81
    WRITE_RSP = 0x82

    HAS_INDEX = READ | WRITE | READ_RSP | WRITE_RSP

    def to_bytes(self) -> bytes:
        return self.value.to_bytes()

    @property
    def has_index(self) -> bool:
        return self.value <= 0x02 or self.value >= 0x81


class pppDatagramError(Exception):
    """Error with the pppDatagram"""


# CANOPEN Abort code lookup table.
# Preceeding word indicates stack. Since we're only using the new stuff, "CANoN, PPP" means ALL.
ABORT_CODES = {
    0x05040000: "ALL: SDO TIMEOUT",
    0x05040001: "CANOPEN: INVALID COMMAND",
    0x05040003: "PPP: OUT OF SEQUENCE WINDOWED WRITE",
    0x05040005: "ALL: OUT OF MEMORY",
    0x06010001: "ALL: READ OF WO OBJECT",
    0x06010002: "ALL: WRITE TO RW OBJECT",
    0x06020000: "ALL: OBJECT DOES NOT EXIST",
    0x06070012: "ALL: DATA TOO LONG",
    0x06070013: "ALL: DATA TOO SHORT",
    0x06090030: "ALL: VALUE OUT OF RANGE",
    0x08000000: "ALL: GENERAL ERROR",
    0x08000020: "ALL: CANNOT STORE TO CONFIG FILE",
}


class pppDatagram:
    PPP_HEADER = b"\xB3\x01"

    def __init__(self, opcode: ppp_opcode, seq_num: int, payload: bytes, windowed: bool = False):
        """Create a datagram. Should not be used to parse a datagram."""
        self.opcode = opcode  # type
        self.flags = b"\x01" if windowed else b"\x00"
        self.seq_num = seq_num
        self.length = len(payload)
        self._payload = payload

        if opcode.has_index():
            self._index = int.from_bytes(payload[0:1], "little")
            self._subindex = int.from_bytes(payload[2], "little")
        else:
            self._index = None
            self._subindex = None

    def encode(self) -> bytes:
        datagram = self.PPP_HEADER + self.PPP_WRITE_OPCODE + self.opcode.to_bytes() + self.flags
        datagram += self.seq_num.to_bytes(2, "little") + self.length.to_bytes()
        datagram += len(self._payload).to_bytes(2, "little")
        datagram += self._payload
        return datagram

    def get_tpdo_vals(self) -> list[[int, int, int | float]] | None:
        if self.opcode != ppp_opcode.TPDO:
            return None

        if self._payload[0] == 0x00:
            return self._tpdo1()
        if self._payload[0] == 0x02:
            return self._tpdo3()
        if self._payload[0] == 0x03:
            return self._tpdo4()
        return None

    def get_heartbeat_info(self) -> [int, int, int]:
        """NMT state u8, period ms u16, boot_id u32"""
        if self.opcode != ppp_opcode.HEARTBEAT:
            return [0,0,0]

        state = self._payload[0]
        period = int.from_bytes(self._payload[1:2], "little")
        boot_id = int.from_bytes(self._payload[3:6], "little")
        return [state, period, boot_id]

    def check_abort(self) -> bool:
        """Gets the read information"""
        if self.opcode != ppp_opcode.READ_RSP and self.opcode != ppp_opcode.WRITE_RSP:
            return False
        code = int.from_bytes(self._payload[2:5], "little")
        return code != 0

    def get_abort_code(self) -> str:
        if self.opcode != ppp_opcode.READ_RSP and self.opcode != ppp_opcode.WRITE_RSP:
            return False
        code = int.from_bytes(self._payload[2:5], "little")
        if code != 0:
            return self._log_abort(code)
        return ""

    def get_data(self) -> bytes:
        if self.opcode != ppp_opcode.READ_RSP:
            raise pppDatagramError("tried to read data from non READ_RSP datagram")
        return self._payload[6:]

    # Constructors
    @classmethod
    def make_read_datagram(cls, seq_num: int, index: int, subindex: int):
        """Helper functioin to make a read datagram with the minimum inputs"""
        payload = index.to_bytes(2, "little") + subindex.to_bytes(1, "little")

        return pppDatagram(ppp_opcode(0x01), seq_num, payload)

    @classmethod
    def make_write_datagram(
        cls,
        seq_num: int,
        index: int,
        subindex: int,
        in_payload: bytes,
        windowed: bool = False
    ):
        """Helper functioin to make a read datagram with the minimum inputs"""
        if len(in_payload) > 1400:
            raise pppDatagramError(f"write datagram length ({len(in_payload)}) exceeds max (1400)")
        payload = index.to_bytes(2, "little") + subindex.to_bytes(1, "little") + in_payload

        return pppDatagram(ppp_opcode(0x01), seq_num, payload, windowed=windowed)

    @classmethod
    def make_heartbeat(cls, seq_num: int):
        """Makes a heartbeat. Hardcoded since if we're not in a good state then we're dead."""
        payload = b"\x05\xE8\x01\x00\x00\x00" # we're fine, 1000ms, session id 1 (LE)
        return pppDatagram(ppp_opcode(0x10), seq_num, payload)

    @classmethod
    def decode(cls, data: bytes):
        """Creates and returns a datagram object based on the """
        if data[0:1] != cls.PPP_HEADER:
            raise pppDatagramError(f"datagram header failed to parse: {data[0:1].hex()}")

        try:
            opcode = ppp_opcode(data[2])
        except ValueError:
            raise pppDatagramError(f"datagram with invalid opcode recieved: {data[2].hex()}")

        flags = data[3]
        seq_num = int.from_bytes(data[4:5], "little")
        payload_len = int.from_bytes(data[6:7], "little")

        if len(data) < payload_len + 8:
            raise pppDatagramError(
                f"payload length {payload_len + 8} is longer than recieved data: {len(data)}"
            )
        payload = data[8:8+payload_len]

        return pppDatagram(opcode,seq_num, payload, windowed=flags)

    @property
    def header(self):
        return self.PPP_HEADER

    @property
    def payload(self):
        return self._payload

    @property
    def index(self) -> int:
        if self.opcode.has_index():
            return self._index
        return -1

    @property
    def subindex(self) -> int:
        if self.opcode.has_index():
            return self._subindex
        return -1

    def _tpdo1 (self) -> list[[int, int, int]]:
        """Returns type information for tpdo1."""
        return list[
            [0x4005, 0x2, int.from_bytes(self._payload[1:4], "little")],
            [0x4005, 0x6, int.from_bytes(self._payload[5:8], "little")]
        ]

    def _tpdo3 (self) -> list[[int, int, float]]:
        """Returns type information for tpdo3."""
        return list[
            [0x4004, 0x4, struct.unpack("<f", self._payload[1:4])],
            [0x4004, 0x5, struct.unpack("<f", self._payload[5:8])]
        ]

    def _tpdo4 (self) -> list[[int, int, int]]:
        """Returns type information for tpdo4."""
        return list[
            [0x4004, 0x1, int.from_bytes(self._payload[1:2], "little")],
            [0x4004, 0x2, int.from_bytes(self._payload[3:4], "little")],
            [0x4004, 0x3, int.from_bytes(self._payload[5:6], "little")]
        ]

    def _log_abort (self, code: int) -> str:
        if code in ABORT_CODES:
            return f"Datagram error code {code}: {ABORT_CODES[code]}"
