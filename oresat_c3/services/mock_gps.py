"""
GPS Mocking Service

Mimics the behavior of the GPS.
"""

from __future__ import annotations

import functools
import json
from dataclasses import dataclass, field
from importlib import resources
from typing import Callable, Optional, TypeVar, Union

import numpy as np
from olaf import Service, logger
from typing_extensions import Concatenate, ParamSpec


@dataclass
class StarTrackerData:
    attitude_known: bool = False
    orientation: np.ndarray = field(default_factory=lambda: np.zeros(4))


@dataclass
class GPSData:
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class IMUData:
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class TimestampedData:
    timestamp: int
    data: Union[StarTrackerData, GPSData, IMUData]


P = ParamSpec("P")
R = TypeVar("R")
S = TypeVar("S")


def gps_callback(
    key: str,
) -> Callable[[Callable[Concatenate[S, P], R]], Callable[Concatenate[S, P], R]]:
    def decorator(func: Callable[Concatenate[S, P], R]) -> Callable[Concatenate[S, P], R]:
        @functools.wraps(func)
        def wrapper(self: S, *args: P.args, **kwargs: P.kwargs) -> R:
            logger.debug("GPS received {} data: {}={}", key, *args)
            buf: Optional[TimestampedData] = self._sensor_data_buffer.get(key, None)
            if not buf:
                buf = TimestampedData(-1, self._data_mapping[key].dataclass())
                self._sensor_data_buffer[key] = buf
                # reset validity buf
                k: str
                for k in self._sensor_data_valid_buffer[key]:
                    self._sensor_data_valid_buffer[key][k] = False
            func(self, *args, **kwargs, buf=buf)
            self._sensor_data_valid_buffer[key][args[0]] = True
            if self._data_buffer_valid(key):
                logger.debug("Data buffer filled for {}", key)
                self._sensor_data[key] = self._sensor_data_buffer.pop(key)

        return wrapper

    return decorator


class MockGPSService(Service):
    def __init__(self) -> None:
        super().__init__()

    def load_state(self) -> None:
        '''Attempts to load the current state from file.'''

        try:
            with resources.path("oresat_c3.data.gps", "mock_gps_state.json") as file_path:
                logger.debug(f"Retrieving file from {file_path}.")
                with open(file_path, 'r') as fd:
                    logger.debug(f"Opened file from {file_path}.")
                    main_state = json.load(fd)
        except Exception:
            logger.error("State file not found!")
            raise

        logger.debug("Loaded state file.")

        # TODO: check if the main state matches the backup state,
        # they should actually match

        # TODO: check if state dictionary is corrupted, it should probably be a dataclass

        logger.debug(f"Loaded state dictionary is: {main_state}")

        self.current_filename = main_state.get("current_filename", None)
        self.dataframe_number = main_state.get("dataframe_number", None)

        if self.current_filename is None:
            logger.warning("")

    def on_start(self) -> None:
        # Recover or initialize state
        self.iter = 0

        # temporary variable to determine if there is loaded data in memory
        self._finished_loaded_data = False
        # temporary variable to determine if there is an unused file to be loaded
        self._file_is_available = False

        # load state, if file is available
        self.load_state()

        # Check if there is data in current data
        # this means the service was apparently in the middle of using data
        # from the files found here
        with resources.path("oresat_c3.data.gps.dataframe_files", "data.csv") as fd:
            self.current_data = np.genfromtxt(fd, delimiter=",")

        # recover the row number
        self.dataframe_number = 0

        # finish
        logger.info("GPS Mocker is ready.")

    def on_loop(self) -> None:
        self.iter += 1
        self.sleep(1)

        logger.debug("Start of loop of GPS Mocker.")

        # STEP 1
        # Determine if GPS card is supposed to be on or off
        # If the GPS is supposed to be on, choose to mimic heartbeats?
        # Listen to OPD line.

        # STEP 2
        # Check if there is loaded data available

        if self.dataframe_number > self.current_data.shape[0]:
            # the dataframe number should not be larger than
            logger.warning("Dataframe number larger than number of rows in loaded data.")

        self._finished_loaded_data = self.dataframe_number >= self.current_data.shape[0]

        if self._finished_loaded_data:
            logger.debug("Loaded data appears to be used, but no new data has been loaded yet.")
            # If no available data in the loaded data (it is all used)
            logger.debug("Checking for new file...")
            logger.error("File checking is not implemented yet!")
            # Archive it
            if self._file_is_available:
                logger.debug("New file found, saving state...")
                # If there is a new batch of data that can be loaded,
                # load the next batch of data.

                # TODO:
                # Step 1: copy the current data to the used folder
                # Step 2: copy the new file (already found) to the current data

                pass
            else:
                # There is no data, do nothing.
                logger.warning("No new data file found, waiting for 1 minute...")
                duration_s = 1 * 60
                self.sleep(duration_s)
                return

        logger.debug(f"Dataframe {self.dataframe_number + 1} of {self.current_data.shape[0]}.")
        dataframe = self.current_data[self.dataframe_number, :]
        logger.debug(f"Retreived gps dataframe: {dataframe}")

        self.dataframe_number += 1
        logger.debug(f"Incremented dataframe_number to {self.dataframe_number}")

        # STEP 3
        # Send the next frame of loaded data

        # time_since_midnight should be a requirement for updating
        # but apparently it is not in the callback
        # self.node._write_cbs[("gps", "skytraq_time_since_midnight")](self.iter)

        # check the dataset size
        self.node._write_cbs[("gps", "skytraq_ecef_x")](dataframe[0])
        self.node._write_cbs[("gps", "skytraq_ecef_y")](dataframe[1])
        self.node._write_cbs[("gps", "skytraq_ecef_z")](dataframe[2])
        self.node._write_cbs[("gps", "skytraq_ecef_vx")](dataframe[3])
        self.node._write_cbs[("gps", "skytraq_ecef_vy")](dataframe[4])
        self.node._write_cbs[("gps", "skytraq_ecef_vz")](dataframe[5])

        # STEP 4
        # Update the state and save it

        if self.dataframe_number == self.current_data.shape[0]:
            logger.warning("Used last dataframe is loaded data.")

    def _data_buffer_valid(self, index: str) -> bool:
        """
        Check if the data buffer at an index contains valid data.
        Parameters
        ----------
        index
            The data buffer index.

        Returns
        -------
        bool
            True if the buffer has valid data, else False.

        """
        return all(b for b in self._sensor_data_valid_buffer[index].values())

    @gps_callback("gps")
    def _on_gps_data(self, subindex: str, value: float, buf: TimestampedData) -> None:
        if not isinstance(buf.data, GPSData):
            return
        if subindex == "skytraq_time_since_midnight":
            buf.timestamp = value
        elif subindex == "skytraq_ecef_x":
            buf.data.position[0] = value
        elif subindex == "skytraq_ecef_y":
            buf.data.position[1] = value
        elif subindex == "skytraq_ecef_z":
            buf.data.position[2] = value
        elif subindex == "skytraq_ecef_vx":
            buf.data.velocity[0] = value
        elif subindex == "skytraq_ecef_vy":
            buf.data.velocity[1] = value
        elif subindex == "skytraq_ecef_vz":
            buf.data.velocity[2] = value
        else:
            logger.error("Received invalid GPS subindex")
