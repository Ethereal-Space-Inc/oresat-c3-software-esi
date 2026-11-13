"""
Payload Service:
Manage the operation of a given payload, Will be designed for OSIRIS first, but with the intention
of extending it to other payloads later.
"""

# unused imports will be used for piplasma and osiris.
import subprocess
import time
from pathlib import Path
from queue import SimpleQueue
from threading import Event, Thread
from time import monotonic

import serial
from canopen.objectdictionary import ODVariable
from canopen.sdo.exceptions import SdoError
from cfdppy import CfdpState
from cfdppy.mib import (
    RemoteEntityConfig,
    RemoteEntityConfigTable,
)
from cfdppy.request import PutRequest
from olaf import MasterNode, Service, logger
from oresat_configs.constants import Mission
from spacepackets.cfdp import (
    ChecksumType,
    TransmissionMode,
)
from spacepackets.util import ByteFieldU8

from ..protocols.cachestore import CacheStore
from ..protocols.cfdp import (
    SourceEntityHandler,
)
from ..subsystems.opd import OpdNode
from .edl import EdlService
from .node_manager import NodeManagerService
from .radios import RadiosService


class PayloadService(Service):
    BAT_LEVEL_LOW = 6500
    BAT_LEVEL_HIGH = 7500

    def __init__(
        self,
        radio_mgr: RadiosService,
        edl_srvce: EdlService,
        node_mgr: NodeManagerService,
        mission: Mission,
        mock: bool = True
    ) -> None:
        super().__init__()
        self._node_mgr = node_mgr
        self._radios = radio_mgr
        self._edl = edl_srvce

        self._mission = mission
        self._mock = mock
        self._payload_handler = None
        # event used to centrally tell payload services to powersave / resume
        self._power_indicatior_event = Event()
        self._power_indicatior_event.clear()
        self._downlink_thread = None

    def on_start(self) -> None:
        time.sleep(5)

        self._state = self.node.od["payload_ctrl"]["state"]
        self._file_register = self.node.od["payload_ctrl"]["file_register"]
        self._enabled = self.node.od["payload_ctrl"]["enabled"]

        self._sdr_should_enable = self.node.od["sdr_ctrl"]["should_enable"]
        self._sdr_sband_downlink = self.node.od["sdr_ctrl"]["sband_downlink"]

        bat_1_rec = self.node.od["battery_1"]
        self._vbatt_bp1_obj = bat_1_rec["pack_1_vbatt"]
        self._vbatt_bp2_obj = bat_1_rec["pack_2_vbatt"]

        self._downlink_handler = None

        if self._mission.__str__() == "osiris_b1":
            logger.info("creating osiris payload handler")
            self._payload_handler = OsirisHandler(
                self._state,
                self._file_register,
                self._node_mgr,
                self.node.fwrite_cache,
                self._power_indicatior_event,
                self._mock,
                self.node
            )
            self._downlink_handler = OsirisDownlink(
                self.node.fread_cache,
                self._radios,
                self._edl,
                self._sdr_should_enable,
                self._sdr_sband_downlink
            )
        elif self._mission.__str__() == "osiris_c":
            logger.info("creating osiris-c payload handler - Empty right now")
            # self._payload_handler = BeeconHandler(self._state)
        elif self._mission.__str__() == "prism":
            logger.info("creating prism payload handler")
            self._payload_handler = PiPlasmaHandler(
                self._state,
                self._node_mgr,
                self.node.fwrite_cache,
                self._power_indicatior_event,
                self._mock,
            )
        elif self._mission.__str__() == "beecon":
            logger.info("creating beecon payload handler")
            self._payload_handler = BeeconHandler(
                self._state,
                self.node.od["beacon"]["delay"],
                self._power_indicatior_event,
                self._mock,
            )
        else:
            logger.error("Payload Service started despite mission not having a compatable payload.")
            if not self._enabled.value or self._state.value != 0:
                self._enabled.value = False
                self._state.value = 0

        if self._downlink_handler is not None:
            self.node.add_sdo_callbacks("payload_ctrl", "downlink_now", None, self.downlink_now)

        if self._vbatt_bp1_obj.value < 2000 and self._vbatt_bp2_obj.value < 2000:
            logger.error("Battery is not giving coherent data!")

    def __del__(self) -> None:
        if self._downlink_thread is not None and self._downlink_thread.is_alive():
            self._downlink_end.set()
            self._downlink_thread.join()

    def on_loop(self) -> None:
        # todo: make sure this works as intended
        if not self._enabled.value and self._state.value == 0:
            time.sleep(2)
            return
        if self._payload_handler is not None:
            try:
                self._payload_handler.loop()
            except Exception as e:
                logger.error(f"Payload handler got error: {e}")
                self._payload_handler = None
        else:
            time.sleep(2)

        if self._power_indicatior_event.is_set() and self.power_high():
            logger.warning("clearing power indicator")
            self._power_indicatior_event.clear()
        elif not self._power_indicatior_event.is_set() and self.power_low():
            logger.warning("setting power indicator")
            self._power_indicatior_event.set()

    def power_low(self):
        """Returns true if the battery is below the low threshold"""
        return (
            self._vbatt_bp1_obj.value < self.BAT_LEVEL_LOW
            and self._vbatt_bp2_obj.value < self.BAT_LEVEL_LOW
        )

    def power_high(self):
        """Returns true if the battery is above the high threshold"""
        return (
            self._vbatt_bp1_obj.value > self.BAT_LEVEL_HIGH
            and self._vbatt_bp2_obj.value > self.BAT_LEVEL_HIGH
        )

    def downlink_now(self, val: bool):
        if self._radios.sband is not None:
            self._downlink_end = Event()
            self._downlink_end.clear()
            self._downlink_thread = Thread(
                target = self._downlink_handler.handle_downlink,
                kwargs={"kill_downlink": self._downlink_end}
            )
            self._downlink_thread.start()

class BeeconHandler:
    # Beecon state pseudoenum:
    # 0: off
    # 1: on
    _I2C_BUS_NUM = 2
    _BEECON_DELAY = 10

    def __init__(
        self, in_state: ODVariable, oresat_beacon_timeout: ODVariable, pwr_event: Event, mock: bool
    ) -> None:
        self._state = in_state
        self._ore_beacon = oresat_beacon_timeout
        self._ore_beacon_default = self._ore_beacon.value
        self._pwr_event = pwr_event

        self._beecon_node = OpdNode(self._I2C_BUS_NUM, "beecon", 0x10, mock=mock)
        self._beecon_node.configure()
        self.failed = False
        if not self._beecon_node.probe():
            logger.error("Beecon handler could not find science card!")
            self.failed = True

    def loop(self) -> None:
        """Runs the beecon state machine. Makes sure the beecon is on or off, depending on state"""
        if self.failed:
            time.sleep(60)
            return
        state_val = self._state.value
        if state_val == 0:
            if self._beecon_node.is_enabled:
                self._ore_beacon.value = self._ore_beacon_default
                self._beecon_node.disable()
        elif state_val == 1:
            if not self._beecon_node.is_enabled:
                self._ore_beacon.value = 0
                self._beecon_node.enable()
            self._check_powersave()
        elif state_val == 2:
            if self._beecon_node.is_enabled:
                self._ore_beacon.value = self._ore_beacon_default
                self._beecon_node.disable()
            self._check_powergood()
        else:
            logger.error("beecon service got incoherent state")
        time.sleep(self._BEECON_DELAY)

    def _check_powersave(self):
        if self._pwr_event.is_set():
            self._state.value = 2

    def _check_powergood(self):
        if not self._pwr_event.is_set():
            self._state.value = 1


class PiPlasmaHandler:
    THRESHOLD = 524288  # 2^19, ~1 orbit of data.

    def __init__(
        self,
        in_state: ODVariable,
        node_mgr: NodeManagerService,
        store: CacheStore,
        pwr_event: Event,
        mock: bool,
    ):
        self._state = in_state
        self._node_mgr = node_mgr
        self._store = store
        self._filesize = 0
        self._file = None
        self._piplasma = None
        self._pwr_event = pwr_event
        self._mock = mock

        if self._state.value > 1:
            self._state.value = 1

    def __del__(self):
        if (
            self._node_mgr.node_status("piplasma_sci") == 1
            or self._node_mgr.node_status("piplasma_sci") == 2
        ):
            self._node_mgr.disable("piplasma_sci")

    def loop(self):
        if self._mock:
            time.sleep(2)
            return

        state_val = self._state.value
        if state_val == 0:
            self._idle()
        elif state_val == 1:
            self._boot_piplasma()
        elif state_val == 2:
            self._process_input()
        elif state_val == 3:
            self._powersave()
        time.sleep(0.1)

    def _idle(self):
        if (
            self._node_mgr.node_status("piplasma_sci") == 1
            or self._node_mgr.node_status("piplasma_sci") == 2
        ):
            self._node_mgr.disable("piplasma_sci")
        if self._piplasma is not None:
            self._piplasma.close()
            self._piplasma = None
        time.sleep(1)

    def _boot_piplasma(self):
        sci_status = self._node_mgr.node_status("piplasma_sci")
        if sci_status == 1:  # wait for the card to boot.
            time.sleep(1)
        elif sci_status == 2:  # on. Goto state 2.
            self._piplasma = serial.Serial(port="/dev/ttyS3", baudrate=115200)
            self._state.value = 2
        elif sci_status == 4 or sci_status == 0xFF:  # nothing to do.
            time.sleep(10)
        else:  # turn the card on.
            self._node_mgr.enable("piplasma_sci")

    def _process_input(self):
        self._handle_file()
        if self._piplasma is None:
            logger.error("Piplasma reached state 2 before state 1!")
            self._state.value = 1
            return
        while self._piplasma.in_waiting > 72:  # it may be better
            self._handle_file()
            out = self._piplasma.read_until(expected=b"\n")
            self._store.write_data(self._file, out, offset=0, from_what=2)
            self._filesize += len(out)

        # should we powersave?
        if self._pwr_event.is_set():
            self._state.value = 3
            self._idle()

    def _powersave(self):
        if not self._pwr_event.is_set():
            self._state.value = 1

    def _handle_file(self) -> None:
        if self._file is None:
            self._make_file()
            logger.info(
                f"Piplasma payload handler has no active file. Creating new file: {self._file}"
            )
        elif self._filesize > self.THRESHOLD:
            self._filesize = 0
            self._make_file()
            logger.info(
                f"Piplasma payload file has reached threshold. Creating new file: {self._file}"
            )

    def _make_file(self):
        timestamp = int(monotonic())
        new_file = Path(f"c3_piplasma_{timestamp}.txt")
        while self._store.file_exists(path=new_file):
            timestamp += 1
            new_file = Path(f"c3_piplasma_{timestamp}.txt")
        self._store.create_file(new_file)
        self._file = new_file


class OsirisHandler:
    def __init__(
        self,
        in_state: ODVariable,
        file_register: ODVariable,
        node_mgr: NodeManagerService,
        read_store: CacheStore,
        pwr_event: Event,
        mock: bool,
        node: MasterNode
    ):
        self._state = in_state
        self._node_mgr = node_mgr
        self._read_store = read_store
        self._pwr_event = pwr_event
        self._file_register = file_register
        self._mock = mock
        self._node = node
        self._timeout = 0
        self._last_state_val = 0
        self._ppp_proc = None

        if self._state.value > 1:
            self._state.value = 1

    def __del__(self):
        if self._ppp_proc is not None:
            self._ppp_proc.kill()

    def loop(self):
        if self._mock:
            time.sleep(2)
            return

        # self._pwr_event

        state_val = self._state.value
        if state_val == 0 and self._last_state_val != 0:
            state_val = 4

        if state_val == 0:
            self._idle()
        if state_val == 1:
            self._startup()
        if state_val == 2:
            self._bootup()
        if state_val == 3:
            self._active()
        if state_val == 4:
            self._shutdown()
        if state_val == 5:
            self._powersave()
        if state_val == 0xFE or state_val == 0xFF:
            time.sleep(1)
        self._last_state_val = state_val

    def _idle(self):
        if (
            self._node_mgr.node_status("osiris_sci") == 1
            or self._node_mgr.node_status("osiris_sci") == 2
        ):
            self._node_mgr.disable("osiris_sci")

    def _startup(self):
        """Turns on the Card. If the card dies, go to the dead mode (does nothing) otherwise cont"""
        while self._node_mgr.node_status("osiris_sci") == 1:
            time.sleep(0.1)
        if self._node_mgr.node_status("osiris_sci") == 0xFF:  # dead
            self._state.value = 0xFF
            return
        if self._node_mgr.node_status("osiris_sci") != 2:
            time.sleep(10)
            return
        self._state.value = 2

    def _bootup(self):
        """
        If we're wating for a component to turn on, wait. If not, handle transitioning to the next
        phase. Fails to the 0xFE state if we lose contact with the science card, requiring manual
        recovery.
        """
        while self._timeout > 0:
            self._timeout -= 1
            time.sleep(1)
            return

        current_state = self._get_bootstate()
        if current_state == 0:  # turn on the Oxco, wait 3 minutes for warming.
            self._change_bootup(1, 180)
        elif current_state == 1:  # turn on the septentrio. Allow 15 sec to boot.
            self._change_bootup(2, 15)
        elif current_state == 2:
            self._change_bootup(3, 5)
        elif current_state == 3:
            self._activate_pppd()
            self._state.value = 3

    def _active(self):
        # have we been commanded to send a file to the osiris?
        if self._file_register.value != "":
            self._send_file_to_picozed()
            self._file_register.value = ""

        # should we powersave?
        if self._pwr_event.is_set():
            self._state.value = 5
            self._idle()

    def _shutdown(self):
        # At some point in the future, we should send a command to the osiris to dump all data now.
        self._node_mgr.disable("osiris_sci")
        self._state.value = 0

    def _powersave(self):
        self._node_mgr.disable("osiris_sci")

        if not self._pwr_event.is_set():
            self._state.value = 1

    def _activate_pppd(self):
        self._ppp_proc = subprocess.Popen(
            "/usr/sbin/pppd",
            "nodetach",
            "nocrtscts",
            "lock",
            "noauth",
            "maxfail",
            "0",
            "persist",
            "10.0.41.1:10.0.41.2",
            "passive",
            "local",
            "noproxyarp",
            "/dev/ttyS3",
            "921600",
        )

    def _change_bootup(self, new_state: int, timeout: int) -> None:
        self._timeout = 5
        res = self._set_bootstate(2)
        if not res:
            self._state.value = 0xFE

    def _set_bootstate(self, new_state: int) -> bool:
        try:
            self._node.sdo_write("osiris_sci", 0x4001, 0x2, new_state)
        except SdoError as e:
            logger.error(f"failed to send sdo command : {e}")
            return False
        return True

    def _get_bootstate(self, new_state: int) -> int:
        try:
            value = self._node.sdo_read("osiris_sci", 0x4001, 0x2)
        except SdoError as e:
            logger.error(f"failed to send sdo command : {e}")
            return -1
        return value

    def _send_file_to_picozed(self) -> None:
        filename = self._file_register.value
        cmd = f"scp -O /var/cache/oresat/fwrite/{filename} root@10.0.41.2:/tmp/uploads"
        subprocess.run(cmd)
        # TODO: Add better reporting


class OsirisDownlink:
    """Class intended to handle managing the downlink through the SDR. Works with but independent of
    the OsirisHandler. Has its own cfdp source entity."""

    def __init__(
        self,
        fread_cache: CacheStore,
        radios: RadiosService,
        edlsrv: EdlService,
        should_enable: ODVariable,
        downlink_through_sband: ODVariable
    ):
        """Create the cfdp source entity."""

        self._fread = fread_cache
        self._radios = radios
        self._edlsrv = edlsrv
        self._od_sdr_enable = should_enable
        self._od_sdr_downlk = downlink_through_sband

        self._payload_put_req_queue = SimpleQueue()
        self.GND_ID = ByteFieldU8(10)
        self.SAT_ID = ByteFieldU8(20)
        self._event = Event()
        self._event.clear()

        remote_entities = RemoteEntityConfigTable(
            [
                RemoteEntityConfig(
                    entity_id=self.GND_ID,
                    max_file_segment_len=None,
                    max_packet_len=966,
                    closure_requested=True,
                    crc_on_transmission=False,
                    default_transmission_mode=TransmissionMode.ACKNOWLEDGED,
                    crc_type=ChecksumType.MODULAR,  # Yamcs only supports the legacy modular crc.
                    immediate_nak_mode=False,
                    nak_timer_interval_seconds=5.0,
                ),
            ]
        )

        self._cfdp_source_handler = SourceEntityHandler(
            self._payload_put_req_queue,
            self._edlsrv.payload_cfdp_src_queue,
            self._edlsrv.payload_cfdp_tm_queue,
            fread_cache,
            remote_entities,
            self.GND_ID,
            self.SAT_ID,
            self._event,
        )
        # self._cfdp_source_handler.start()

    def _create_file_archive(self) -> str:
        """
        Take all of the files from the picozed (.bz2 in the filename somewhere) and put them into
        an archive. We don't compress the arhive because all the component files are already
        compressed.
        """

        # get the files from the fread directory, reduce it to the ones that use bz2 (from picozed)
        files = self._fread.data
        files_to_downlink = [file.name for file in files if "bz2" in file.name]

        # create a new tar archive and put all of the files into it.
        new_archive = f"osiris_archive_{time.time()}.tar"
        cmd = f"tar -rf {new_archive} {' '.join(files_to_downlink)}"
        logger.info(f"Running cmd \"{cmd}\" to prepare archive for downlink.")
        subprocess.run(cmd, shell=True, cwd=self._fread.dir)

        # delete the files that are now in the archive.
        # for file in files_to_downlink[:]:
        #     self._fread.delete_file(OreSatFile(file))

        return new_archive

    def _create_pdu(self, file: str):
        """Create a pdu from the given str as a filename and put it into the source handler."""
        self._transmission_bytes = self._fread.stat(Path(file))

        # create the PDU
        put_req = PutRequest(
            destination_id=self.GND_ID,
            source_file=Path(file),
            dest_file=Path(file),
            trans_mode=TransmissionMode.UNACKNOWLEDGED, # for testing, blind transfer
            closure_requested=False
        )

        self._payload_put_req_queue.put(put_req)

    def handle_downlink(self, kill_downlink: Event):
        """Master function that should be called when trying to downlink."""

        # turn on the sband
        self._od_sdr_enable.value = True
        prev_sdr_downlink = self._od_sdr_downlk.value
        self._od_sdr_downlk.value = False
        time.sleep(3)

        # create the archive the raspi was
        archive = self._create_file_archive()

        # await the S band being on.
        # while not self._radios.sdr_ready():
        #     time.sleep(1)
            # if kill_downlink.is_set():
            #     return

        # Turn on the source handler and create the PDU.
        self._cfdp_source_handler.start()
        self._create_pdu(archive)
        time.sleep(5)

        # await the src_handler entering idle mode then stop it
        while not self._cfdp_source_handler.source_handler.state == CfdpState.IDLE:
            time.sleep(1)
            if kill_downlink.is_set():
                return
        self._event.set()
        time.sleep(2)
        self._event.clear()

        # await the sband having no more frames to send and having returned to the ready state.
        # while (not self._radios.sband_queue.empty()) or (not self._radios.sband.can_send_now()):
        #     time.sleep(1)
            # if kill_downlink.is_set():
            #     return

        # turn off the sband
        self._od_sdr_enable.value = False
        self._od_sdr_downlk.value = prev_sdr_downlink
