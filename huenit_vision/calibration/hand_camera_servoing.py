"""
hand_camera_servoing.py – Hand‑Kamera Visual‑Servoing
Implements calibration des Referenzpixels und iterative Feinjustierung.
"""

import logging
import time
import math
from typing import Tuple, Optional

from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.config import RobotConfig
from huenit_vision.aruco import detect_marker_0
from huenit_vision.workspace import check_reachable

logger = logging.getLogger(__name__)

class HandCameraServoing:
    """Visual‑Servoing mit der Handkamera.

    * ``calibrate_reference`` ermittelt das Pixel‑Ziel (U,V) für ein
      bekannten Referenzpunkt (x_ref, y_ref) in der Höhe ``z_verify``.
    * ``fine_adjust`` führt bis zu ``servoing_max_steps`` iterative
      Korrekturen durch, bis die Pixel‑Abweichung kleiner als
      ``servoing_tolerance_px`` ist.
    """

    def __init__(self, conn: GCodeConnection, cameras: CameraManager, config: RobotConfig):
        self.conn = conn
        self.cameras = cameras
        self.config = config
        self._target_pixel: Optional[Tuple[int, int]] = None

    # ---------------------------------------------------------------------
    def calibrate_reference(self, x_ref: float, y_ref: float) -> Tuple[int, int]:
        """Bewege den Arm zum Referenzpunkt und bestimme das Marker‑Pixel‑Ziel.

        Der Arm wird zu (x_ref, y_ref, z_verify) gefahren, kurz stabilisiert
        und dann ein Bild der Handkamera aufgenommen. Das gefundene
        Pixel‑Paar wird gespeichert und zurückgegeben.
        """
        z_verify = self.config.z_verify
        logger.debug(f"Calibrating hand‑camera reference at ({x_ref}, {y_ref}, {z_verify})")
        self.conn.move(x_ref, y_ref, z_verify, self.config.feed_servoing)
        time.sleep(self.config.camera_settle_time)
        frame = self.cameras.capture_hand()
        if frame is None:
            raise RuntimeError("Failed to capture frame from hand camera during calibration")
        pixel = detect_marker_0(frame)
        if pixel is None:
            raise RuntimeError("Marker ID 0 not found in hand‑camera calibration frame")
        self._target_pixel = pixel
        logger.info(f"Hand‑camera reference pixel: {pixel}")
        return pixel

    # ---------------------------------------------------------------------
    def fine_adjust(self, x_target: float, y_target: float) -> Optional[Tuple[float, float]]:
        """Iterative Position‑Korrektur bis zur Pixel‑Toleranz.

        Parameter
        ----------
        x_target, y_target : float
            Soll‑Position in mm (auf gleicher Z‑Höhe ``z_verify``).
        Returns
        -------
        (x_final, y_final) : Tuple[float, float] | None
            Letzte erreichbare Position, wenn das Verfahren konvergiert
            oder die Ziel‑Pixel‑Toleranz nicht erreicht wird. ``None`` wird
            zurückgegeben, wenn der Marker nicht mehr sichtbar ist oder die
            Position außerhalb des Arbeitsraums liegt.
        """
        if self._target_pixel is None:
            raise RuntimeError("Hand‑camera servoing not calibrated – call calibrate_reference first")
        U_target, V_target = self._target_pixel
        x_curr, y_curr = x_target, y_target
        z_verify = self.config.z_verify
        for step in range(self.config.servoing_max_steps):
            # Bewege zu aktueller Position und warte auf Abschluss
            if not self.conn.move_and_verify(x_curr, y_curr, z_verify, self.config.feed_servoing):
                logger.warning(f"Position ({x_curr}, {y_curr}) nicht erreichbar bei Schritt {step}")
                return None
            time.sleep(self.config.camera_settle_time)
            frame = self.cameras.capture_hand()
            if frame is None:
                logger.error("Hand‑camera capture failed during fine adjust")
                return None
            p = detect_marker_0(frame)
            if p is None:
                logger.warning("Marker nicht mehr sichtbar während fine adjust")
                return None
            err_u = p[0] - U_target
            err_v = p[1] - V_target
            dist_px = math.hypot(err_u, err_v)
            logger.debug(f"Step {step}: pixel error (u={err_u}, v={err_v}), dist={dist_px:.2f}px")
            if dist_px < self.config.servoing_tolerance_px:
                logger.info(f"Fine adjust converged after {step+1} steps")
                return (x_curr, y_curr)
            # Gewinne aus Gains
            gain_x, gain_y = self.config.get_servoing_gains(z_verify)
            dx = gain_x * err_u
            dy = gain_y * err_v
            # Begrenze Schrittgröße
            dx = max(min(dx, self.config.servoing_step_limit_mm), -self.config.servoing_step_limit_mm)
            dy = max(min(dy, self.config.servoing_step_limit_mm), -self.config.servoing_step_limit_mm)
            x_curr += dx
            y_curr += dy
            # Sicherheits‑Check
            if not check_reachable(x_curr, y_curr, z_verify):
                logger.warning("Fine adjust would leave reachable workspace – aborting")
                return None
        logger.info(f"Fine adjust max steps reached, returning last position ({x_curr}, {y_curr})")
        return (x_curr, y_curr)
