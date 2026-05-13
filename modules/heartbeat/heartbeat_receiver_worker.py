"""
Heartbeat worker that sends heartbeats periodically.
"""

import os
import pathlib

from pymavlink import mavutil

from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import heartbeat_receiver
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def heartbeat_receiver_worker(
    connection: mavutil.mavfile,
    heartbeat_period: float,
    disconnect_threshold: int,
    error_tolerance: float,
    output_queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    args... describe what the arguments are
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (heartbeat_receiver.HeartbeatReceiver)
    result, heartbeat_receiver_instance = heartbeat_receiver.HeartbeatReceiver.create(
        connection,
        heartbeat_period,
        disconnect_threshold,
        error_tolerance,
        local_logger,
    )
    if not result:
        local_logger.error("Failed to create HeartbeatReceiver", True)
        return

    assert heartbeat_receiver_instance is not None

    # Main loop: do work.
    while not controller.is_exit_requested():
        controller.check_pause()

        result, state, missed_heartbeat = heartbeat_receiver_instance.run()
        if not result:
            local_logger.error("Failed to read heartbeat from connection", True)
            continue

        if missed_heartbeat:
            local_logger.warning("Missed heartbeat", True)

        output_queue.queue.put(state)
        local_logger.info(f"Connection state: {state}", True)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
