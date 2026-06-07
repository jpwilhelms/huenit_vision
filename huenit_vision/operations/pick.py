"""
pick.py – Pick‑Logik für Huenit Vision
Implements the robust pick routine with visual servoing and verification.
"""

import logging
import time
from typing import Tuple

from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.calibration.hand_camera_servoing import HandCameraServoing
from huenit_vision.aruco import detect_marker_0
from huenit_vision.config import RobotConfig

logger = logging.getLogger(__name__)

def pick_object(
    conn: GCodeConnection,
    cameras: CameraManager,
    servoing: HandCameraServoing,
    x: float,
    y: float,
    Z_pick: float,
    Z_travel: float,
    config: RobotConfig,
) -> bool:
    """Attempt to pick an object at (x, y).

    Returns ``True`` when the object is successfully removed from the
    tabletop (i.e. the marker is no longer visible from the overhead camera).
    ``False`` is returned after ``config.max_pick_retries`` failed attempts.
    """
    for attempt in range(1, config.max_pick_retries + 1):
        logger.info(f"Pick attempt {attempt} for point ({x}, {y})")
        # --- Feinjustierung mit Handkamera ---
        try:
            fine = servoing.fine_adjust(x, y)
        except Exception as e:
            logger.error(f"Fine‑adjust failed: {e}")
            fine = None
        if fine is None:
            logger.warning("Fine‑adjust failed – falling back to nominal coordinates")
            x_adj, y_adj = x, y
        else:
            x_adj, y_adj = fine

        # --- Absenken und prüfen ob Bewegung möglich ---
        if not conn.move_and_verify(x_adj, y_adj, Z_pick, config.feed_vertical):
            logger.warning("Absenken blockiert – versuchen, in sichere Höhe zu fahren")
            conn.move(x_adj, y_adj, config.z_verify, config.feed_vertical)
            continue

        # --- Vakuum aktivieren ---
        conn.vacuum_on()
        time.sleep(config.vacuum_buildup_time)

        # --- Anheben ---
        conn.move(x_adj, y_adj, Z_travel, config.feed_vertical)

        # --- Verifikation über Deckenkamera ---
        conn.move(config.park_x, config.park_y, Z_travel, config.feed_travel)
        time.sleep(config.camera_settle_time)
        frame = cameras.capture_overhead()
        if frame is None:
            logger.error("Failed to capture overhead frame for pick verification")
            # treat as failure, continue attempts
            conn.vacuum_off(blow_off=False)
            continue
        marker = detect_marker_0(frame)
        if marker is None:
            # Marker nicht mehr sichtbar → Pick erfolgreich
            logger.info("Pick erfolgreich – Marker nicht mehr sichtbar")
            return True
        else:
            # Marker noch sichtbar → Pick fehlgeschlagen
            logger.warning("Pick fehlgeschlagen – Marker noch sichtbar, vakuum ausschalten ohne Blas‑Off")
            conn.vacuum_off(blow_off=False)
            # continue to next attempt
            continue
    # alle Versuche fehlgeschlagen
    logger.error("Alle Pick‑Versuche fehlgeschlagen")
    return False
