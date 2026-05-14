"""
Decision-making logic.
"""

import math

from pymavlink import mavutil

from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        height_tolerance: float,
        z_speed: float,
        angle_tolerance_degrees: float,
        turning_speed_degrees_per_second: float,
        local_logger: logger.Logger,
    ) -> "tuple[bool, Command | None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        if connection is None:
            local_logger.error("Command requires a MAVLink connection", True)
            return False, None
        if target is None:
            local_logger.error("Command requires a target position", True)
            return False, None
        if height_tolerance < 0.0:
            local_logger.error("Height tolerance must be non-negative", True)
            return False, None
        if z_speed <= 0.0:
            local_logger.error("Z speed must be greater than 0", True)
            return False, None
        if angle_tolerance_degrees < 0.0:
            local_logger.error("Angle tolerance must be non-negative", True)
            return False, None
        if turning_speed_degrees_per_second <= 0.0:
            local_logger.error("Turning speed must be greater than 0", True)
            return False, None

        return (
            True,
            Command(
                cls.__private_key,
                connection,
                target,
                height_tolerance,
                z_speed,
                angle_tolerance_degrees,
                turning_speed_degrees_per_second,
                local_logger,
            ),
        )

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        height_tolerance: float,
        z_speed: float,
        angle_tolerance_degrees: float,
        turning_speed_degrees_per_second: float,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.height_tolerance = height_tolerance
        self.z_speed = z_speed
        self.angle_tolerance_degrees = angle_tolerance_degrees
        self.turning_speed_degrees_per_second = turning_speed_degrees_per_second
        self.local_logger = local_logger

        self.velocity_samples = 0
        self.sum_x_velocity = 0.0
        self.sum_y_velocity = 0.0
        self.sum_z_velocity = 0.0

    def run(
        self,
        telemetry_data: telemetry.TelemetryData,
    ) -> "tuple[bool, str | None]":
        """
        Make a decision based on received telemetry data.
        """
        # Log average velocity for this trip so far
        if (
            telemetry_data.x_velocity is None
            or telemetry_data.y_velocity is None
            or telemetry_data.z_velocity is None
        ):
            self.local_logger.error("Telemetry velocity is incomplete", True)
            return False, None

        self.velocity_samples += 1
        self.sum_x_velocity += telemetry_data.x_velocity
        self.sum_y_velocity += telemetry_data.y_velocity
        self.sum_z_velocity += telemetry_data.z_velocity

        avg_x_velocity = self.sum_x_velocity / self.velocity_samples
        avg_y_velocity = self.sum_y_velocity / self.velocity_samples
        avg_z_velocity = self.sum_z_velocity / self.velocity_samples
        self.local_logger.info(
            (
                "Average velocity so far: "
                f"({avg_x_velocity}, {avg_y_velocity}, {avg_z_velocity}) m/s"
            ),
            True,
        )

        # Use COMMAND_LONG (76) message, assume the target_system=1 and target_componenet=0
        # The appropriate commands to use are instructed below

        # Adjust height using the comand MAV_CMD_CONDITION_CHANGE_ALT (113)
        # String to return to main: "CHANGE_ALTITUDE: {amount you changed it by, delta height in meters}"
        if telemetry_data.z is None:
            self.local_logger.error("Telemetry z is missing", True)
            return False, None

        delta_z = self.target.z - telemetry_data.z
        if abs(delta_z) > self.height_tolerance:
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                0,
                self.z_speed,
                0,
                0,
                0,
                0,
                0,
                self.target.z,
            )
            return True, f"CHANGE ALTITUDE: {delta_z}"

        # Adjust direction (yaw) using MAV_CMD_CONDITION_YAW (115). Must use relative angle to current state
        # String to return to main: "CHANGING_YAW: {degree you changed it by in range [-180, 180]}"
        # Positive angle is counter-clockwise as in a right handed system
        if telemetry_data.x is None or telemetry_data.y is None or telemetry_data.yaw is None:
            self.local_logger.error("Telemetry x/y/yaw is missing", True)
            return False, None

        target_yaw_radians = math.atan2(
            self.target.y - telemetry_data.y,
            self.target.x - telemetry_data.x,
        )
        relative_yaw_radians = target_yaw_radians - telemetry_data.yaw
        relative_yaw_radians = (relative_yaw_radians + math.pi) % (2 * math.pi) - math.pi
        relative_yaw_degrees = math.degrees(relative_yaw_radians)

        if abs(relative_yaw_degrees) > self.angle_tolerance_degrees:
            self.connection.mav.command_long_send(
                1,
                0,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                relative_yaw_degrees,
                self.turning_speed_degrees_per_second,
                0,
                1,
                0,
                0,
                0,
            )
            return True, f"CHANGE YAW: {relative_yaw_degrees}"

        return True, None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
