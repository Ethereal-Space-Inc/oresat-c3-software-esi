"""
Radios Service

Handles interfacing with the radio driver daemon.
"""

import socket
import struct
import time
from math import ceil
from queue import SimpleQueue
from time import monotonic

from canopen.objectdictionary import ODVariable
from canopen.sdo.exceptions import SdoError
from gpiod.line import Value
from olaf import MasterNode, Service, logger

from ..drivers.si41xx import Si41xx, Si41xxIfdiv
from ..subsystems._gpio import request_gpio_input, request_gpio_output
from .node_manager import NodeManagerService


# TODO: The sband is expecting a message before it sends. we don't do this, and theres some
# scheduling that we need to do to make sure that things work.
class RadiosService(Service):
    """Radios Service."""

    BEACON_DOWNLINK_ADDR = ("localhost", 10015)
    EDL_UPLINK_ADDR = ("localhost", 10025)
    EDL_DOWNLINK_ADDR = ("localhost", 10016)
    UHF_RSSI_ADDR = ("localhost", 10030)
    BUFFER_LEN = 4096

    def __init__(self, node_mgr: NodeManagerService, mock_hw: bool = False):
        """
        Request gpio, initialize radios, add daemons, and create message queue.

        Parameters
        ----------
        mock_hw : bool
            Flag to enable hardware mocking. True if enabled.
        """
        super().__init__()

        self._mock_hw = mock_hw
        self.enable_uhf = False
        if mock_hw:
            self.uhf = Radio()
            self.lband = Radio()
        else:
            self._radio_enable_gpio = request_gpio_output("/dev/gpiochip1", 22, "RADIO_ENABLE")
            self.uhf = UHFRadio()
            self.lband = Radio()

        self.recv_queue: SimpleQueue[bytes] = SimpleQueue()
        self._node_mgr = node_mgr

        self.sband_queue: SimpleQueue[bytes] = SimpleQueue()
        self._sband_ready_at = monotonic()

    def on_start(self):
        """Provide uninterruptible power-on sequence, and bring up radio daemons."""
        logger.info("enabling radio power domain")
        if not self._mock_hw:
            self._radio_enable_gpio.set_value(self._radio_enable_gpio.offsets[0], Value.ACTIVE)
            time.sleep(0.1)
        self.node.add_daemon("lband")
        self.node.add_daemon("uhf")

        logger.info("enabling uhf radio")
        self.uhf.enable()
        logger.info("enabling lband radio")
        self.lband.enable()

        self.node.od["lband"]["synth_relock_count"].value = self.lband.rf_reset_count

        # FIXME: add an OD for UHF TOT clear count

        # beacon downlink: UDP client
        logger.info(f"Beacon socket: {self.BEACON_DOWNLINK_ADDR}")
        self._beacon_downlink_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # EDL uplink: UDP server
        logger.info(f"EDL uplink socket: {self.EDL_UPLINK_ADDR}")
        self._edl_uplink_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._edl_uplink_socket.bind(self.EDL_UPLINK_ADDR)
        self._edl_uplink_socket.settimeout(1)

        # EDL downlink: UDP client
        logger.info(f"EDL downlink socket: {self.EDL_DOWNLINK_ADDR}")
        self._edl_downlink_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # EDL downlink: UDP client
        logger.info(f"UHF RSSI socket: {self.UHF_RSSI_ADDR}")
        self._uhf_rssi_socket = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM | socket.SOCK_NONBLOCK
        )
        self._uhf_rssi_socket.bind(self.UHF_RSSI_ADDR)

        if not self._mock_hw:
            self.node.daemons["uhf"].start()
            self.node.daemons["lband"].start()

        self._sband_downlink = self.node.od["sdr_ctrl"]["sband_downlink"]
        self._sband_enable = self.node.od["sdr_ctrl"]["should_enable"]
        self._sband_timeout = self.node.od["sdr_ctrl"]["enable_timeout"]
        self._sband_status = self.node.od["sdr_ctrl"]["status"]
        self.node.add_sdo_callbacks("sdr_ctrl", "should_enable", None, self._sband_should_enable_cb)

        if "sdr" in self.node._od_db:  # not self._mock_hw and
            logger.info("creating sband radio class")
            self.sband = SbandRadio(self.node, self._node_mgr, self._sband_status)
        else:
            self.sband = None

    def on_loop(self):
        """Maintain radio health and receive edl requests."""
        if not self.uhf.is_rf_ok():
            logger.error("tot okay was low, resetting radios")
            self.uhf.rf_reset()
            # FIXME: Add OD TOT counter

        lBandOk = self.lband.is_rf_ok()
        self.node.od["lband"]["synth_lock"].value = lBandOk

        if not lBandOk:
            logger.error("si41xx unlocked, resetting lband synth")
            self.lband.rf_reset()
            self.node.od["lband"]["synth_relock_count"].value = self.lband.rf_reset_count

        if self.sband is not None and (self._sband_enable.value or self.sband.state != 0):
            self._handle_sband()

        if recv := self._recv_edl_request():
            self.recv_queue.put(recv)
        try:
            rssi, src = self._uhf_rssi_socket.recvfrom(128)
        except OSError:
            pass
        else:
            try:
                self.node.od["uhf"]["rssi"].value = struct.unpack('b', rssi)[0]
            except struct.error as e:
                logger.error(f"Invalid RSSI paylaod: {e}")
            logger.debug(f"UHF rssi: {rssi} from {src}")

    def on_stop(self):
        """Power down radios and stop daemons."""
        logger.info("disabling radios")
        if not self._mock_hw:
            self.node.daemons["lband"].stop()
            self.node.daemons["uhf"].stop()

        self._beacon_downlink_socket.close()
        self._edl_downlink_socket.close()
        self._edl_uplink_socket.close()
        self._uhf_rssi_socket.close()

        # power down sequence
        logger.info("disabling uhf radio")
        self.uhf.disable()

        logger.info("disabling lband radio")
        self.lband.disable()

        logger.info("disabling radio power domain")
        if not self._mock_hw:
            self._radio_enable_gpio.set_value(self._radio_enable_gpio.offsets[0], Value.INACTIVE)

    def send_edl_response(self, message: bytes):
        """
        Send an EDL packet.

        Parameters
        ----------
        message : bytes
            The message to send as a byte string.
        """
        if self._downlink_through_sband():
            self.try_send_sband(message)
            logger.debug(f"Sent EDL downlink packet through sband: {message.hex(sep=' ')}")
        else:
            try:
                self._edl_downlink_socket.sendto(message, self.EDL_DOWNLINK_ADDR)
            except Exception as e:  # pylint: disable=W0718
                logger.error(f"failed to send mess over EDL downlink: {e}")

            logger.debug(f"sent EDL downlink packet: {message.hex(sep=' ')}")

    def send_beacon(self, message: bytes):
        """
        Send a beacon.

        Parameters
        ----------
        message : bytes
            The beacon to beacon.
        """

        if self._downlink_through_sband():
            self.try_send_sband(message)
            logger.debug(f"Sent beacon downlink packet through sband: {message.hex(sep=' ')}")
        else:
            try:
                self._beacon_downlink_socket.sendto(message, self.BEACON_DOWNLINK_ADDR)
            except Exception as e:  # pylint: disable=W0718
                logger.error(f"failed to send beacon message: {e}")

            logger.debug(f"Sent beacon downlink packet: {message.hex(sep=' ')}")

    def _recv_edl_request(self) -> bytes:
        """
        Recieve an EDL packet.

        Returns
        -------
        bytes
            The EDL packet or empty byte string if nothing is received.
        """
        try:
            message, src = self._edl_uplink_socket.recvfrom(self.BUFFER_LEN)
        except socket.timeout:
            return b""
        logger.debug(f"received EDL uplink packet: {message.hex(sep=' ')} from {src}")

        return message

    #### SBAND HANDLING

    def _downlink_through_sband(self) -> bool:
        """Can/should we downlink through the sband?"""
        if self.sband is None:
            return False
        return self._sband_downlink.value and self.sband.is_rf_ok()

    def _sband_should_enable_cb(self, value: bool) -> None:
        """Turns on or off the sband (if it exists) based on passed value"""
        if self.sband is None:
            return
        if value:
            logger.info("Started Sband bootup process.")
            self.sband.enable()
            self._sband_timeout_timestamp = monotonic() + self._sband_timeout.value
        else:
            self.sband.disable()
            self._sband_timeout_timestamp = 0
            logger.info("Disabled Sband")

    def _handle_sband(self) -> None:
        """Don't like this function. Should work for now."""
        if self.sband is None:
            return

        if not self._sband_enable.value and self.sband._state:
            self._sband_enable.value = False
            # I don't think this will auto call the enable callback. TODO: confirm this.
            self._sband_should_enable_cb(False)
            return

        if self._sband_enable.value and not self.sband.is_rf_ok():
            if self.sband.state == 0xFF:
                self._sband_enable.value = False
                # I don't think this will auto call the enable callback. TODO: confirm this.
                self._sband_should_enable_cb(False)
                return
            self.sband.enable()

        if self._sband_timeout_timestamp < monotonic():
            logger.info("Sband timeout reached.")
            self._sband_enable.value = False
            # I don't think this will auto call the enable callback. TODO: confirm this.
            self._sband_should_enable_cb(False)
            return
        # this function has 3 instances of code reuse. Bad. That should be fixed.

        if (
            self._sband_ready_at < monotonic()
            and not self.sband_queue.empty()
            and not self.sband_lock
        ):
            self._send_queued()

        # if we have sent frames to the sdr and it has been 1 second since the last frame, downlink.
        if self._sband_send_time > 0 and self.last_frame + 1 > monotonic():
            self.force_sband_send()

    def force_sband_send(self) -> None:
        """Send the cmd to start sending data, and compute the time after which we will be done"""
        self.sband.send_queued_messages()
        self._sband_ready_at = monotonic() + self._sband_send_time
        self._sband_send_time = 0

    def try_send_sband(self, message: bytes) -> None:
        """
        Function to try to send bytes to sband. If the sband is currently busy, send the bytes to
        the queue instead.
        """
        if self._sband_ready_at > monotonic():
            self.sband_queue.put(message)
        else:
            ### BAD. SHOULD TIMEOUT.
            while self.sband_lock:
                time.sleep(0.033)
            if not self.sband_queue.empty():
                self._send_queued()
            self._send_on_sband(message)

    def _send_on_sband(self, message: bytes) -> None:
        """
        Send the given bytes to the sband. Do not use this for generic sending, as it DOES NOT check
        that the sband is ready for more bytes. Use this and only this to send data to the sdr.
        """
        if self.sband is None:
            return
        try:
            sband_rnode = self.node.remote_nodes["sdr"]
            with sband_rnode.sdo['tx_data'].open(
                'wb', size=len(message), block_transfer=True
            ) as outfile:
                outfile.write(message)
                self._add_sband_time(len(message))
                self.last_frame = monotonic()
        except SdoError as e:
            logger.error(f"failed to send data to sband: {e}")

    def _add_sband_time(self, msg_len: int) -> None:
        """
        Computes and adds the est time that the sband will take to finish.
        SDR sends fixed size frames with payload size ~1097 octets. We will be using data rate
        675000 spSec / 2 spSym, so 675000 bps.
        """
        if msg_len < 1097:
            self._sband_send_time += 1275 / 675000
        else:
            frames = ceil(msg_len / 1097)
            self._sband_send_time += frames * (1275 / 675000)

    def _send_queued(self) -> None:
        self.sband_lock = True
        try:
            while not self.sband_queue.empty():
                frame = self.sband_queue.get()
                self._send_on_sband(frame)
        except Exception as e:
            self.sband_lock = False
            self.sband_queue.queue.clear()
            logger.error(f"failed to send all elements of the sband queue: {e}")
        self.sband_lock = False


class Radio:
    def __init__(self):
        self._rf_reset_count = 0

    def enable(self):
        pass

    def disable(self):
        pass

    def is_rf_ok(self) -> bool:
        return True

    def rf_reset(self):
        self._rf_reset_count += 1

    @property
    def rf_reset_count(self):
        return self._rf_reset_count


class LBandRadio(Radio):
    """Provides production implmentation of the L-band radio subsystem."""

    def __init__(self) -> None:
        """
        Initialize L-band synth and request gpio.

        See Also
        --------
        oresat_c3.drivers.si41xx : Driver for the L-band synth.
        """
        super().__init__()
        self._si41xx = Si41xx(
            sen_pin="LBAND_LO_nSEN",
            sclk_pin="LBAND_LO_SCLK",
            sdata_pin="LBAND_LO_SDATA",
            auxout_pin="LBAND_LO_nLOCKED",
            ref_freq=16_000_000,  # Hz
            if_div=Si41xxIfdiv.DIV1,
            if_n=1616,
            if_r=32,
            mock=False,
        )

        # request gpio pins
        self._lband_enable_gpio = request_gpio_output("/dev/gpiochip0", 19, "LBAND_ENABLE")

    def enable(self):
        """
        Enable L-band power domain and start L-band synth.

        Notes
        -----
        The radio power domain must be enabled first.
        """
        self._lband_enable_gpio.set_value(self._lband_enable_gpio.offsets[0], Value.ACTIVE)
        time.sleep(0.1)
        self._si41xx.start()

    def disable(self):
        """Stop L-band synth and disable L-band power domain."""
        self._si41xx.stop()

        self._lband_enable_gpio.set_value(self._lband_enable_gpio.offsets[0], Value.INACTIVE)
        time.sleep(0.1)

    def is_rf_ok(self) -> bool:
        """Check if the L-band synth is locked.

        Returns
        -------
        bool
            True if the L-band is locked.
        """
        # si41xx_nlock is active low
        return not self._si41xx.aux()

    def rf_reset(self):
        """Reset the L-band synth."""
        # increment reset counter
        super().rf_reset()
        self._si41xx.stop()
        self._si41xx.start()


class UHFRadio(Radio):
    """Provides production implmentation of the UHF radio subsystem."""

    # in seconds
    TOT_CLEAR_DELAY = 0.01

    def __init__(self):
        """Request gpio."""
        super().__init__()
        self._uhf_tot_ok_gpio = request_gpio_input("/dev/gpiochip0", 25, "UHF_TOT_OK")
        self._uhf_tot_clear_gpio = request_gpio_output("/dev/gpiochip0", 26, "UHF_TOT_CLEAR")
        self._uhf_enable_gpio = request_gpio_output("/dev/gpiochip0", 16, "UHF_ENABLE")
        self._uhf_enable_pa = request_gpio_output("/dev/gpiochip0", 27, "UHF_PA_ENABLE")

    def enable(self):
        """
        Enable UHF power domain and clear hardware time out timer (TOT).

        Notes
        -----
        The radio power domain must be enabled first.
        """
        self._uhf_enable_gpio.set_value(self._uhf_enable_gpio.offsets[0], Value.ACTIVE)
        self._uhf_enable_pa.set_value(self._uhf_enable_pa.offsets[0], Value.ACTIVE)
        time.sleep(0.1)

        # clear timeout timer
        self._uhf_tot_clear()

    def disable(self):
        """Disable the UHF power domain."""
        self._uhf_enable_gpio.set_value(self._uhf_enable_gpio.offsets[0], Value.INACTIVE)
        self._uhf_enable_pa.set_value(self._uhf_enable_pa.offsets[0], Value.INACTIVE)
        time.sleep(0.1)

    def is_rf_ok(self) -> bool:
        """
        Check if the hardware timeout timer (TOT) is ok.

        Returns
        -------
        bool
            True if the UHF TOT is ok
        """
        return bool(self._uhf_tot_ok_gpio.get_value(self._uhf_tot_ok_gpio.offsets[0]))

    def rf_reset(self):
        """Reset the UHF radio."""
        # increment reset counter
        super().rf_reset()
        self.disable()
        self.enable()

    def _uhf_tot_clear(self):
        """Clear TOT."""
        self._uhf_tot_clear_gpio.set_value(self._uhf_tot_clear_gpio.offsets[0], Value.ACTIVE)
        time.sleep(self.TOT_CLEAR_DELAY)

        self._uhf_tot_clear_gpio.set_value(self._uhf_tot_clear_gpio.offsets[0], Value.INACTIVE)


class SbandRadio(Radio):
    # self.state pseudoenum:
    # 0: off
    # 1: on, no heartbeat
    # 2: on, heartbeat, sent powerup radio message
    # 3: on, heartbeat, ready to transmit. is_rf_ok returns true.
    # 4: graceful shutdown.
    # 255: error.

    SDO_FAULT_LIMIT = 5  # number of failed sdos before going to 0xFF state.

    def __init__(self, node: MasterNode, node_mgr: NodeManagerService, status: ODVariable):
        """Request gpio."""
        super().__init__()
        self._node = node
        self._node_mgr = node_mgr
        self._state = 0
        self._od_status = status

    def enable(self) -> None:
        """State machine for the bootup process."""
        if self._state == 0:
            logger.debug("telling nodemgr to turn on sdr.")
            self._node_mgr.enable("sdr")
            self._state = 1
            self._sdo_fault = 0

        elif self._state == 1:
            node_status = self._node_mgr.node_status("sdr")
            logger.debug(f"nodemgr status of sdr: {node_status}.")
            if node_status == 2:  # The sdr is powered on.
                self._send_start_cmd()
                self._state = 2
            elif node_status == 0xFF:  # dead
                self._state = 0xFF
                self.status.value = self._state
                self.disable()
                logger.warning("sband handler was told that sdr is dead.")

        elif self._state == 2:
            val = self._get_sdr_status()
            if val == 0:
                logger.info("resending power on cmd to sdr.")
                self._send_start_cmd()
            elif val == 2:
                self._state = 3
            elif val == 0xFF:
                self._state = 0xFF
                self.status.value = self._state
                self.disable()
                logger.warning("sband handler sdoread error from sdr.")

        else:
            logger.warning(f"Sband radio enable called with invalid state: {self._state}.")
            self._state = 0xFF

        time.sleep(0.25)

    def disable(self) -> None:
        self._state = 4
        self._sdo_fault = 0
        self._shutdown()
        self._state = 0

    def _shutdown(self) -> None:
        self._send_stop_cmd()
        time.sleep(5)
        self._node_mgr.disable("sdr")

    # fault tolerant sdo functions.
    def _send_start_cmd(self) -> None:
        try:
            self._node.sdo_write("sdr", "sdr_power", None, 1)
        except SdoError as e:
            logger.error(f"failed to send sdo power on cmd to SDR: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF

    def _send_stop_cmd(self) -> None:
        try:
            self._node.sdo_write("sdr", "sdr_power", None, 0)
        except SdoError as e:
            logger.error(f"failed to send sdo power off cmd to SDR: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF

    def _get_sdr_status(self) -> int:
        try:
            val = self._node.sdo_read("sdr", "sdr_status", None)
        except SdoError as e:
            logger.error(f"failed to get sdr status: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF
            val = 0
        return val

    def send_queued_messages(self) -> int:
        """The sband needs us to send a 1 to 0x400B before it begins downlinking data"""
        try:
            self._node.sdo_write("sdr", "tx_send_cache", None, 1)
        except SdoError as e:
            logger.error(f"failed to get sdr status: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF

    def is_rf_ok(self) -> bool:
        self._od_status.value = self._state
        return self._state == 3

    def rf_reset(self) -> None:
        self._rf_reset_count += 1

    @property
    def rf_reset_count(self):
        return self._rf_reset_count

    @property
    def state(self):
        return self._state
