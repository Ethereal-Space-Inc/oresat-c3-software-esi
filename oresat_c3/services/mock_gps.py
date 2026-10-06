"""
GPS Mocking Service

Mimics the behavior of the GPS.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass, field
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

    def on_start(self) -> None:
        # Recover or initialize state
        self.iter = 0

        # temporary variable to determine if there is loaded data in memory
        self._data_is_available = True
        # temporary variable to determine if there is an unused file to be loaded
        self._file_is_available = False

        # load data
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
        if not self._data_is_available:
            # If no available data in the loaded data (it is all used)
            # Archive it
            if self._file_is_available:
                # If there is a new batch of data that can be loaded,
                # load the next batch of data.
                pass
            else:
                # There is no data, do nothing.
                return

        # STEP 3
        # Send the next frame of loaded data

        # time_since_midnight should be a requirement for updating
        # but apparently it is not in the callback
        # self.node._write_cbs[("gps", "skytraq_time_since_midnight")](self.iter)

        self.node._write_cbs[("gps", "skytraq_ecef_x")](20)
        self.node._write_cbs[("gps", "skytraq_ecef_y")](20)
        self.node._write_cbs[("gps", "skytraq_ecef_z")](20)
        self.node._write_cbs[("gps", "skytraq_ecef_vx")](20)
        self.node._write_cbs[("gps", "skytraq_ecef_vy")](20)
        self.node._write_cbs[("gps", "skytraq_ecef_vz")](20)

        # STEP 4
        # Update the state and save it

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
