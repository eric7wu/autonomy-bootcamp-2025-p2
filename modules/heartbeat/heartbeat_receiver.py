"""
Heartbeat receiving logic.
"""

from pymavlink import mavutil

from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        heartbeat_period: float,
        disconnect_threshold: int,
        error_tolerance: float,
        local_logger: logger.Logger,
    ) -> "tuple[bool, HeartbeatReceiver | None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        if connection is None:
            local_logger.error("HeartbeatReceiver requires a MAVLink connection", True)
            return False, None
        if heartbeat_period <= 0.0:
            local_logger.error("Heartbeat period must be greater than 0", True)
            return False, None
        if disconnect_threshold <= 0:
            local_logger.error("Disconnect threshold must be greater than 0", True)
            return False, None
        if error_tolerance < 0.0:
            local_logger.error("Error tolerance must be non-negative", True)
            return False, None

        return (
            True,
            HeartbeatReceiver(
                cls.__private_key,
                connection,
                heartbeat_period,
                disconnect_threshold,
                error_tolerance,
            ),
        )

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        heartbeat_period: float,
        disconnect_threshold: int,
        error_tolerance: float,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self.connection = connection
        self.heartbeat_timeout = heartbeat_period + error_tolerance
        self.disconnect_threshold = disconnect_threshold
        self.missed_heartbeats = 0
        self.connection_state = "Disconnected"

    def run(
        self,
    ) -> "tuple[bool, str, bool]":
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        try:
            msg = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=self.heartbeat_timeout,
            )
        except (AttributeError, RuntimeError, OSError, ValueError):
            return False, self.connection_state, False

        if msg and msg.get_type() == "HEARTBEAT":
            self.missed_heartbeats = 0
            self.connection_state = "Connected"
            return True, self.connection_state, False

        self.missed_heartbeats += 1
        if self.missed_heartbeats >= self.disconnect_threshold:
            self.connection_state = "Disconnected"

        return True, self.connection_state, True


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
