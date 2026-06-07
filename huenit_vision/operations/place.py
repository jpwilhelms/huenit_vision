"""
place.py – Place‑Logik für Huenit Vision
Implements the place routine with safe release and optional blow‑off.
"""

import logging
import time

from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.config import RobotConfig

logger = logging.getLogger(__name__)

def place_object(
    conn: GCodeConnection,
    x: float,
    y: float,
    Z_place: float,
    Z_travel: float,
    config: RobotConfig,
) -> bool:
    """Place an object at (x, y).

    Returns ``True`` when the placement sequence completes successfully.
    ``False`` on any movement failure.
    """
    # 1. Move to travel height above target
    if not conn.move_and_verify(x, y, Z_travel, config.feed_travel):
        logger.error("Failed to move to travel height for placement")
        return False
    # 2. Move down to place height
    if not conn.move_and_verify(x, y, Z_place, config.feed_vertical):
        # leave vacuum on, raise error, but try to retract
        logger.error("Failed to move to place height – retracting")
        conn.move(x, y, Z_travel, config.feed_vertical)
        return False
    # 3. Release vacuum (with blow‑off)
    conn.vacuum_off(blow_off=True)
    # 4. Retract to travel height
    conn.move(x, y, Z_travel, config.feed_vertical)
    logger.info("Place operation completed successfully")
    return True
