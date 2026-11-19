"""
This class should eventually be used to define a mission database. For now, it will be used to
store gps and solar information.
"""

from os.path import abspath
from time import monotonic, time

import canopen
from olaf import Service, logger, new_oresat_file, scet_int_to_time

from .node_manager import NodeManagerService


class MissionDatabaseService(Service):
    """Mission Database Service"""

    BAT_LEVEL_HIGH = 7500
    BAT_LEVEL_LOW = 6500  # mV

    def __init__(self, node_mgr_service: NodeManagerService) -> None:
        super().__init__()
        self._node_mgr_service = node_mgr_service

        self.next_gps_index = 1
        self._gps_data = ""
        self.current_datapoints = 0

        self._b1_p1_voltage: list[canopen.objectdictionary.Variable] = None
        self._b1_p2_voltage: list[canopen.objectdictionary.Variable] = None
        self._b2_p1_voltage: list[canopen.objectdictionary.Variable] = None
        self._b2_p2_voltage: list[canopen.objectdictionary.Variable] = None

    def on_start(self) -> None:
        self._od_active = self.node.od["mdb"]["active"]
        self._od_refresh_delay = self.node.od["mdb"]["refresh_delay"]
        self._od_max_num_files = self.node.od["mdb"]["max_num_files"]
        self._od_data_per_file = self.node.od["mdb"]["data_per_file"]
        self._od_active_timeout = self.node.od["mdb"]["active_timeout"]
        self._od_idle_timeout = self.node.od["mdb"]["idle_timeout"]
        self._last_gps_timeout = monotonic()

        self._init_battery_od()
        self._init_power_od()
        self._init_gps_od()

    def on_loop(self) -> None:
        """"""
        if not self._od_active.value:
            self.sleep(1)
            return

        try:
            self._handle_gps()
            self._record_power()
        except Exception as e:
            logger.error(f"MDB service experienced following issue!: {e}")
            self._od_active.value = False
        self.sleep(1)

    ### Battery Helpers
    def _init_battery_od(self) -> None:
        if "battery_1" in self.node._od_db.keys():
            bat_1_rec = self.node.od["battery_1"]
            self._b1_p1_voltage = bat_1_rec["pack_1_vbatt"]
            self._b1_p2_voltage = bat_1_rec["pack_2_vbatt"]
        if "battery_2" in self.node._od_db.keys():
            bat_2_rec = self.node.od["battery_2"]
            self._b2_p1_voltage = bat_2_rec["pack_1_vbatt"]
            self._b2_p2_voltage = bat_2_rec["pack_2_vbatt"]

    @property
    def is_bat_lvl_good(self) -> bool:
        """bool: Helper property to check if the battery levels are above low."""
        good = True
        if "battery_1" in self.node._od_db.keys():
            good = good and self._node_mgr_service.node_status("battery_1") != 0xFF
            good = good and self._b1_p1_voltage.value > self.BAT_LEVEL_LOW
            good = good and self._b1_p2_voltage.value > self.BAT_LEVEL_LOW
        if "battery_2" in self.node._od_db.keys():
            good = good and self._node_mgr_service.node_status("battery_2") != 0xFF
            good = good and self._b2_p1_voltage.value > self.BAT_LEVEL_LOW
            good = good and self._b2_p2_voltage.value > self.BAT_LEVEL_LOW
        return good

    ### Power Functions
    def _init_power_od(self) -> None:
        """
        Creates the list of power ODvars to be used. Values should be preceeded by the python
        time.time()
        """
        self._power_header = "time,"
        self._ods_power_system = []

        # k gives the name of the card, which is also used for generating the rpdo receiving point
        for k in self.node.od_db.keys():
            if "solar" in k:
                item = self.node.od[k]["output_current"]
                self._ods_power_system.append(item)
                self._power_header += f"{k} current,"
            elif "battery" in k:
                item = self.node.od[k]["pack_1_current"]
                self._ods_power_system.append(item)
                self._power_header += f"{k} p1Current,"
                item = self.node.od[k]["pack_2_current"]
                self._ods_power_system.append(item)
                self._power_header += f"{k} p2Current,"
                item = self.node.od[k]["pack_1_vbatt"]
                self._ods_power_system.append(item)
                self._power_header += f"{k} p1Voltage,"
                item = self.node.od[k]["pack_2_vbatt"]
                self._ods_power_system.append(item)
                self._power_header += f"{k} p2Voltage,"

        self._power_file_data = ""
        self._power_file_data += self._power_header + "\n"
        self._power_data_start = monotonic()

    def _record_power(self) -> None:
        line = f"{int(time())},"
        for var in self._ods_power_system:
            line += f"{var.value},"
        self._power_file_data += line + "\n"
        self._handle_power_file()

    def _handle_power_file(self) -> None:
        """Hardcode the power file to take 90 min of data."""
        if self._power_data_start + 54 > monotonic():
            return

        new_file_name = new_oresat_file("powercsv", "c3", -1, ".csv")
        new_file_path = abspath(self.node.cache_base_dir + new_file_name)
        logger.info(f"Making file {new_file_name}")
        with open(new_file_path, "w") as f:
            f.write(self._power_file_data)

        self.node.fwrite_cache.add(new_file_path, True)
        self._power_file_data = ""
        self._power_file_data += self._power_header + "\n"
        self._power_data_start = monotonic()

        files = self.node.fwrite_cache.files("powercsv")
        while len(files) > self._od_max_num_files.value:
            files = sorted(files)
            logger.warning(f"Deleting file {files[0].name}")
            self.node.fwrite_cache.delete_file(files[0])
            files = self.node.fwrite_cache.files("powercsv")

        self.current_datapoints = 0

    ### GPS Functions
    def _init_gps_od(self) -> None:
        """Create the objects to easily reference gps information"""
        self._ecef_x = self.node.od["gps"]["skytraq_ecef_x"]
        self._ecef_y = self.node.od["gps"]["skytraq_ecef_y"]
        self._ecef_z = self.node.od["gps"]["skytraq_ecef_z"]
        self._ecef_vx = self.node.od["gps"]["skytraq_ecef_vx"]
        self._ecef_vy = self.node.od["gps"]["skytraq_ecef_vy"]
        self._ecef_vz = self.node.od["gps"]["skytraq_ecef_vz"]
        self._scet = self.node.od["gps"]["scet"]

    def _handle_gps(self) -> None:
        if (
            self._node_mgr_service.node_status("gps") == 0xFF
            or self._node_mgr_service.node_status("gps") == 4
        ):  # Dead or missing
            return

        if self._last_gps_timeout < monotonic():
            return
        self._set_csv_gps()
        self._set_od_gps()
        self._update_gps_file()

        if ( # shut off if low on power or we are out of the capture window.
            not self.is_bat_lvl_good
            or monotonic() + self._gps_enabled_time > self._od_active_timeout.value
        ):
            self._gps_should_on = False
        elif self.is_bat_lvl_good and self._gps_idle_time > self._od_idle_timeout.value:
            self._gps_should_on = True

    def _manage_gps(self) -> None:
        if (
            self._gps_should_on
            and self._node_mgr_service.node_status("gps") != 1  # On.
            and self._node_mgr_service.node_status("gps") != 2  # Boot
        ):
            self._node_mgr_service.enable("gps")
            self._enabled_time = monotonic()
        elif (
            not self._gps_should_on
            and self._node_mgr_service.node_status("gps") != 1  # On.
            and self._node_mgr_service.node_status("gps") != 2  # Boot
        ):
            self._node_mgr_service.disable("gps")
            self._gps_idle_time = monotonic()

    def _update_gps_file(self) -> None:
        if self.current_datapoints < self._od_data_per_file.value:
            return

        new_file_name = new_oresat_file("gps-data", "c3", -1, ".csv")
        new_file_path = abspath(self.node.cache_base_dir + new_file_name)
        logger.info(f"Making file {new_file_name}")
        with open(new_file_path, "w") as f:
            f.write(self._gps_data)

        self.node.fwrite_cache.add(new_file_path, True)
        self._gps_data = ""

        files = self.node.fwrite_cache.files("gps-data")
        while len(files) > self._od_max_num_files.value:
            files = sorted(files)
            logger.warning(f"Deleting file {files[0].name}")
            self.node.fwrite_cache.delete_file(files[0])
            files = self.node.fwrite_cache.files("gps-data")

        self.current_datapoints = 0

    def _set_csv_gps(self) -> None:
        self._last_gps_timeout = monotonic + self._od_refresh_delay.value

        self.current_datapoints += 1
        self._gps_data += str(self._ecef_x.value) + ","
        self._gps_data += str(self._ecef_y.value) + ","
        self._gps_data += str(self._ecef_z.value) + ","
        self._gps_data += str(self._ecef_vx.value) + ","
        self._gps_data += str(self._ecef_vy.value) + ","
        self._gps_data += str(self._ecef_vz.value) + ","

        time = scet_int_to_time(self._scet.value)
        self._gps_data += str(int(time)) + "\n"

    def _set_od_gps(self) -> None:
        append = f"_{self.next_gps_index}"

        self.node.od["hist_ecef_x"]["ecef_x" + append].value = self._ecef_x.value
        self.node.od["hist_ecef_y"]["ecef_y" + append].value = self._ecef_y.value
        self.node.od["hist_ecef_z"]["ecef_z" + append].value = self._ecef_z.value
        self.node.od["hist_ecef_vx"]["ecef_vx" + append].value = self._ecef_vx.value
        self.node.od["hist_ecef_vy"]["ecef_vy" + append].value = self._ecef_vy.value
        self.node.od["hist_ecef_vz"]["ecef_vz" + append].value = self._ecef_vz.value

        time = scet_int_to_time(self._scet.value)
        self.node.od["hist_unix_time"]["time" + append].value = int(time)
        self.next_gps_index += 1
        if self.next_gps_index > 32:
            self.next_gps_index = 1
