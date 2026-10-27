"""
Radios Service

Handles interfacing with the radio driver daemon.
"""

import socket
import struct
import time
from queue import SimpleQueue
from time import monotonic

from canopen.objectdictionary import ObjectDictionary, ODVariable
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
        self.node.add_sdo_callbacks("sdr_ctrl", "reset", None, self._sband_reset_cb)

        self.sband_lock = False

        if "sdr" in self.node._od_db:  # not self._mock_hw and
            logger.info("creating sband radio class")
            self.sband = SbandRadio(
                self.node,
                self._node_mgr,
                self._sband_status,
                self._sband_timeout,
                self._sband_enable,
                self.node.remote_nodes["sdr"]
            )
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
            self.sband.state_machine()
        else:
            self.sband.state_machine()
            logger.info("Disabled Sband")

    def _sband_reset_cb(self, value: bool) -> None:
        """Turns on or off the sband (if it exists) based on passed value"""
        if self.sband is None:
            return
        self.sband.rf_reset()

    def _handle_sband(self) -> None:
        """Don't like this function. Should work for now."""
        if self.sband is None:
            return
        self.sband.state_machine()

        if (
            self.sband.can_send_now()
            and not self.sband_queue.empty()
            and not self.sband_lock
        ):
            self._send_queued()

    def try_send_sband(self, message: bytes) -> None:
        """
        Function to try to send bytes to sband. If the sband is currently busy, send the bytes to
        the queue instead.
        """
        if not self.sband.can_send_now():
            logger.debug("putting frame for sband in queue.")
            self.sband_queue.put(message)
        else:
            ### BAD. SHOULD TIMEOUT.
            while self.sband_lock:
                time.sleep(0.033)
            logger.debug("sending frame to sband directly.")
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
            self.sband.send_frame(message)
        except SdoError as e:
            logger.error(f"failed to send data to sband: {e}")

    def _send_queued(self) -> None:
        self.sband_lock = True
        try:
            while not self.sband_queue.empty():
                frame = self.sband_queue.get()
                self.sband.send_frame(frame)
                logger.debug(f"sent queued frame to sband. remaining: {self.sband_queue.qsize}")
        except Exception as e:
            self.sband_lock = False
            self.sband_queue.queue.clear()
            logger.error(f"failed to send all elements of the sband queue: {e}")
        self.sband_lock = False

    def force_sband_send(self):
        self.sband.send_data_now()


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

    # The desired samples per second and sample rate for the SBand.
    SET_SAMPLES = 2
    SET_RATE = 675000
    MARGIN = 2

    SDO_FAULT_LIMIT = 5  # number of failed sdos before going to 0xFF state.

    def __init__(
        self,
        node: MasterNode,
        node_mgr: NodeManagerService,
        status: ODVariable,
        timeout: ODVariable,
        should_enable: ODVariable,
        _sdr_rnode: ObjectDictionary
    ):
        """Initialization."""
        super().__init__()
        self._node = node
        self._node_mgr = node_mgr
        self._state = 0
        self._od_status = status
        self._od_timeout = timeout
        self._od_should_enable = should_enable
        self._sdr_rnode = _sdr_rnode

        self._sdo_fault = 0
        self._timeout_start_time = 0
        self._send_timer = monotonic()
        self._frames_to_send = 0
        self._last_frame = 0

    def state_machine(self):
        """State machine for the Sband. States are defined above."""
        if not self._od_should_enable.value and self._state != 0:
            self._state = 4

        # would be really nice if python had switch statements.
        if self._state == 0:
            self._off()
        elif self._state == 1: # Turn on the sband.
            self._boot()
        elif self._state == 2:
            self._on_idle()
        elif self._state == 3:
            self._on_active()
        elif self._state == 4:
            self._disable()
        elif self._state == 0xFF:
            self._fault()

    # State machine functions
    def _off(self):
        """If the sdr is on, turn it off"""
        if self._od_should_enable.value:
            self._state = 1

        node_status = self._node_mgr.node_status("sdr")
        if node_status != 0 and node_status != 0xFF and node_status != 4:
            logger.info("Turning off SDR due to state 0.")
            self._sdo_fault = 0 # we shouldn't error if we're trying to turn it off.
            self._fault_tolerant_forced_lax_write("tx_send_cache", None, 0)
            self._fault_tolerant_forced_lax_write("sdr_power", None, 0)
            self._sdo_fault = 0
            self._node_mgr.disable("sdr")

    def _boot(self):
        """turns on the sband until it heartbeats and has been configured"""
        node_status = self._node_mgr.node_status("sdr")
        logger.debug(f"nodemgr status of sdr: {node_status}.")
        if node_status == 0:
            logger.debug("telling nodemgr to turn on sdr.")
            self._node_mgr.enable("sdr")
            self._timeout_start_time = monotonic()
            self._state = 1
            self._sdo_fault = 0
        elif node_status == 2:  # The sdr is powered on. Configure and move to state 2.
            self._configure()
            self._reset()
            self._state = 2
        elif node_status == 0xFF:  # dead
            self.disable()
            self._state = 0xFF
            self.status.value = self._state
            logger.warning("sband handler was told that sdr is dead.")

    def _on_idle(self):
        """Recieve frames from other services. Once a timeout occurs or a the
        radios.force_sband_send() function is called, create a new timeout for the sband actually
        downlinking the sent information."""
        if self._last_frame + 1 < monotonic() and self._frames_to_send != 0:
            self.send_data_now()

        if self._timeout_start_time + self._od_timeout.value < monotonic():
            self._od_should_enable.value = False
            self._state = 4

    def _on_active(self):
        """Wait for the send timer to run out."""
        if self._send_timer < monotonic():
            self._reset()
            self._state = 2

    def _disable(self) -> None:
        """Turn off the sband."""
        logger.info("Shutting down s-band gracefully")
        self._sdo_fault = 0 # we shouldn't error if we're trying to turn it off.
        self._fault_tolerant_lax_sdo_write("tx_send_cache", None, 0)
        self._fault_tolerant_lax_sdo_write("sdr_power", None, 0)
        self._sdo_fault = 0
        self._node_mgr.disable("sdr")
        time.sleep(2) # give the nodemanager time to mark it as off before state 0 kills it again.
        self._state = 0

    def _fault(self) -> None:
        """Turn off the sband. Do not move from state 0xFF unless reset."""
        node_status = self._node_mgr.node_status("sdr")
        if node_status != 0 and node_status != 0xFF and node_status != 4:
            logger.info("Turning off SDR due to state 0xFF.")
            self._node_mgr.disable("sdr")

    # Helper functions to run several SDO commands in series.
    def _configure(self):
        """
        Sets the configuration in the SDR.
        If it fails, keep trying until we reach a limt or wesucceed.
        """
        # self._node.sdo_write("sdr", "tx_settings", "scrambler_enabled", 0)
        # self._node.sdo_write("sdr", "tx_settings", "idle_enabled", 0)
        # self._node.sdo_write("sdr", "tx_settings", "rs_enabled", 1)

        samples = self._fault_tolerant_forced_sdo_read("tx_settings", "samples_per_symbol")
        if samples[1] and (samples[0] != self.SET_SAMPLES):
            logger.info(f"Setting sdr samples per symbol to {self.SET_SAMPLES} from {samples}")
            self._fault_tolerant_forced_sdo_write(
                "tx_settings",
                "samples_per_symbol",
                self.SET_SAMPLES
            )
        elif not samples[1]:
            self._state = 0xFF
            logger.error("Sband SDR driver failed during _configure in samples per symbol")

        sample_rate = self._fault_tolerant_forced_sdo_read("tx_settings", "sampling_rate")
        if sample_rate[1] and (sample_rate[0] != self.SET_RATE):
            logger.info(f"Setting sdr sample_rate to {self.SET_RATE} from {sample_rate}")
            self._fault_tolerant_forced_sdo_write(
                "tx_settings",
                "sampling_rate",
                self.SET_RATE
            )
        elif not sample_rate[1]:
            self._state = 0xFF
            logger.error("Sband SDR driver failed during _configure in sample rate")

    def _reset(self):
        """send the commands to turn off the Transmitter and let the S-Band take more data."""
        succeeded = True
        succeeded and self._fault_tolerant_forced_sdo_write("sdr_power", None, 0)
        succeeded and self._fault_tolerant_forced_sdo_write("tx_send_cache", None, 0)
        self._frames_to_send = 0
        if not succeeded:
            self._state = 0xFF
            logger.error("Sband SDR driver failed during _reset")

    def send_data_now(self):
        """
        Send the commands to being downlinking and start the timer for when we should send again"""
        self._state = 3
        baud = self.SET_RATE / (self.SET_SAMPLES / 2) # should be 675000 with the flight config
        duration = self._frames_to_send * 1275 * 8 / baud + 1 # SDR uses fixed 1275B frames.
        # We add a second to the duration so that there aren't any issues with state switching.

        succeeded = True
        succeeded and self._fault_tolerant_forced_sdo_write("sdr_power", None, 1)
        succeeded and self._fault_tolerant_forced_sdo_write("tx_send_cache", None, 1)
        if not succeeded:
            self._state = 0xFF
            logger.error("Sband SDR driver failed during send_data_now")

        self._send_timer = monotonic() + duration + self.MARGIN

    # Fault tolerant SDO functions
    def _fault_tolerant_lax_sdo_read_log(
        self,
        index: int | str,
        subindex: int | str | None,
        info: bool = False
    ) -> None:
        """
        Tries to sdo read once and output the read value to the log. Returns nothing. This function
        is intended to be used for creating debug information in the log. By default, it will output
        to logger.debug although if info is True then it will output to logger.info instead.
        """
        try:
            value = self._node.sdo_read("sdr", index, subindex)
        except SdoError as e:
            logger.error(f"failed to sdo read for debugging from SDR: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF
                return
            time.sleep(0.01)
        else:
            self._sdo_fault = 0
            if info:
                logger.info(f"SDR sdoread from {index}:{subindex} returned {value}")
            else:
                logger.debug(f"SDR sdoread from {index}:{subindex} returned {value}")

    def _fault_tolerant_forced_sdo_read(
        self,
        index: int | str,
        subindex: int | str | None,
    ) -> [str | float | bytes | bool | int, bool]:
        """
        Keeps trying to SDO read until it fails too many times or succeeds. Returns the value and
        success state, with the value being meaningful only if the sdo succeeded.
        """
        while True:
            try:
                value = self._node.sdo_read("sdr", index, subindex)
            except SdoError as e:
                logger.error(f"failed to sdo read from SDR: {e}")
                self._sdo_fault += 1
                if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                    self._state = 0xFF
                    return [0, False]
                time.sleep(0.051)
            else:
                logger.debug(f"Read value {value} from {index}:{subindex} on S band SDR")
                self._sdo_fault = 0
                return [value, True]

    def _fault_tolerant_lax_sdo_write(
        self,
        index: int | str,
        subindex: int | str | None,
        value: str | float | bytes | bool
    ) -> bool:
        """
        Keeps trying to read an SDO until it fails too many times or succeeds. Returns the value
        and success state, with the value being meaningful only if the sdo succeeded.
        """
        try:
            self._node.sdo_write("sdr", index, subindex, value)
        except SdoError as e:
            logger.error(f"failed to sdo write SDR: {e}")
            self._sdo_fault += 1
            if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                self._state = 0xFF
                return False
            return False
        else:
            logger.debug(f"Wrote value {value} to {index}:{subindex} on S band SDR")
            self._sdo_fault = 0
            return True

    def _fault_tolerant_forced_sdo_write(
        self,
        index: int | str,
        subindex: int | str | None,
        value: str | float | bytes | bool
    ) -> bool:
        """
        Keeps trying to read an SDO until it fails too many times or succeeds. Returns the value
        and success state, with the value being meaningful only if the sdo succeeded.
        """
        while True:
            try:
                self._node.sdo_write("sdr", index, subindex, value)
            except SdoError as e:
                logger.error(f"failed to sdo write SDR: {e}")
                self._sdo_fault += 1
                if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                    self._state = 0xFF
                    return False
                time.sleep(0.051)
            else:
                logger.debug(f"Wrote value {value} to {index}:{subindex} on S band SDR")
                self._sdo_fault = 0
                return True

    def _fault_tolerant_forced_sdo_block_write_frame(
        self,
        frame: bytes
    ) -> bool:
        """
        Keeps trying to send a given frame until it either succeeds, or fails too much. Uses the
        block transfer sdo rather than the normal, slower SDOs. This method is intended to be used
        only for the transfer of complete frames, and should not be used for anything else.
        """
        if len(frame) > 1024:
            logger.error("Error sending frame to sband, got frame length greater than 1024 bytes.")
            return False
        attempts = 0

        while True:
            try:
                with self._sdr_rnode.sdo['tx_data'].open(
                    'wb', size=len(frame), block_transfer=True
                ) as outfile:
                    outfile.write(frame)
            except SdoError as e:
                logger.error(f"failed to block transfer sdo write frame to sband: {e}")
                self._fault_tolerant_lax_sdo_read_log("tx_data_statistics", "rx_bytes")
                self._sdo_fault += 1
                if self._sdo_fault >= self.SDO_FAULT_LIMIT:
                    if attempts < 2:
                        attempts += 1
                        logger.error(f"reattempting SDO of frame to Sband. Attempts:{attempts}")
                        time.sleep(0.31415) # ideally shift us off any PDOs
                        continue
                    self._state = 0xFF
                    return False
                time.sleep(0.01)
            else:
                logger.debug("Sent frame to S band SDR")
                self._sdo_fault = 0
                self._frames_to_send += 1
                self._last_frame = monotonic()
                return

    # customer facing interaction functions
    def send_frame(self, payload):
        """Just calls the private function."""
        self._fault_tolerant_forced_sdo_block_write_frame(payload)

    def can_send_now(self):
        """Used to determine if we should queue or send immediately"""
        return self._state == 2

    def is_rf_ok(self) -> bool:
        """Used to determine if we should try to use the sband (see can_send_now for how.)"""
        self._od_status.value = self._state
        return self._state == 2 or self._state == 3

    def rf_reset(self) -> None:
        """Turns off the sdr, clears the state, and restarts the boot process."""
        self._node_mgr.disable("sdr")
        time.sleep(5)
        self._state = 1
        self._rf_reset_count += 1

    @property
    def rf_reset_count(self):
        return self._rf_reset_count

    @property
    def state(self):
        return self._state
