"""
Telemetry gathering logic.
"""

import time

from pymavlink import mavutil

from ..common.modules.logger import logger


class TelemetryData:  # pylint: disable=too-many-instance-attributes
    """
    Python struct to represent Telemtry Data. Contains the most recent attitude and position reading.
    """

    def __init__(
        self,
        time_since_boot: int | None = None,  # ms
        x: float | None = None,  # m
        y: float | None = None,  # m
        z: float | None = None,  # m
        x_velocity: float | None = None,  # m/s
        y_velocity: float | None = None,  # m/s
        z_velocity: float | None = None,  # m/s
        roll: float | None = None,  # rad
        pitch: float | None = None,  # rad
        yaw: float | None = None,  # rad
        roll_speed: float | None = None,  # rad/s
        pitch_speed: float | None = None,  # rad/s
        yaw_speed: float | None = None,  # rad/s
    ) -> None:
        self.time_since_boot = time_since_boot
        self.x = x
        self.y = y
        self.z = z
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_velocity = z_velocity
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

    def __str__(self) -> str:
        return f"""{{
            time_since_boot: {self.time_since_boot},
            x: {self.x},
            y: {self.y},
            z: {self.z},
            x_velocity: {self.x_velocity},
            y_velocity: {self.y_velocity},
            z_velocity: {self.z_velocity},
            roll: {self.roll},
            pitch: {self.pitch},
            yaw: {self.yaw},
            roll_speed: {self.roll_speed},
            pitch_speed: {self.pitch_speed},
            yaw_speed: {self.yaw_speed}
        }}"""


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Telemetry:
    """
    Telemetry class to read position and attitude (orientation).
    """

    __private_key = object()
    timeout_seconds = 1.0

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[bool, Telemetry | None]":
        """
        Falliable create (instantiation) method to create a Telemetry object.
        """
        if connection is None:
            local_logger.error("Telemetry requires a MAVLink connection", True)
            return False, None

        return True, Telemetry(cls.__private_key, connection, local_logger)

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Telemetry.__private_key, "Use create() method"

        self.connection = connection
        self.local_logger = local_logger

    def run(
        self,
    ) -> (
        "tuple[bool, TelemetryData | None, bool]"
    ):  # prevent mixed meanings returning a single None
        """
        Receive LOCAL_POSITION_NED and ATTITUDE messages from the drone,
        combining them together to form a single TelemetryData object.
        """
        # Read MAVLink message LOCAL_POSITION_NED (32)
        # Read MAVLink message ATTITUDE (30)
        # Return the most recent of both, and use the most recent message's timestamp
        start_time = time.time()
        remaining_time = self.timeout_seconds
        latest_position = None
        latest_attitude = None

        while remaining_time > 0.0 and (latest_position is None or latest_attitude is None):
            msg = self.connection.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=True,  # prevents the timeout warning from spamming in the logs
                timeout=remaining_time,
            )

            if msg is None:
                break

            if msg.get_type() == "LOCAL_POSITION_NED":
                latest_position = msg
            elif msg.get_type() == "ATTITUDE":
                latest_attitude = msg

            remaining_time = self.timeout_seconds - (time.time() - start_time)

        if latest_position is None or latest_attitude is None:
            self.local_logger.warning(
                "Timed out waiting for ATTITUDE and LOCAL_POSITION_NED pair", True
            )
            return False, None, True

        time_since_boot = max(latest_position.time_boot_ms, latest_attitude.time_boot_ms)
        output = TelemetryData(
            time_since_boot=time_since_boot,
            x=latest_position.x,
            y=latest_position.y,
            z=latest_position.z,
            x_velocity=latest_position.vx,
            y_velocity=latest_position.vy,
            z_velocity=latest_position.vz,
            roll=latest_attitude.roll,
            pitch=latest_attitude.pitch,
            yaw=latest_attitude.yaw,
            roll_speed=latest_attitude.rollspeed,
            pitch_speed=latest_attitude.pitchspeed,
            yaw_speed=latest_attitude.yawspeed,
        )
        return True, output, False


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
