#!/usr/bin/env python3
"""
calibrate_with_huenit_vision.py

Ablauf:
1. Aktuelle X/Y/Z-Position ermitteln (Greif-Position)
2. Z schrittweise erhoehen, bis Aruco Marker 0 erkannt wird
3. Um zusaetzlichen Offset erhoehen fuer optimale Lesehoehe (Token zentriert)
4. Pixel-Koordinaten der Token-Mitte an Lesehoehe speichern
5. Ergebnisse in JSON-Datei sichern
"""

import os
import sys
import time
import json
import logging

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.aruco import detect_marker_0

OUTPUT_DIR = os.path.join(PROJECT_ROOT, "calib_results")
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "token_alignment.json")

# Parameter fuer die Hoehensuche
Z_STEP_MM = 5.0
Z_MAX_MM = 160.0
ADDITIONAL_HEIGHT_MM = 20.0


def setup_logging() -> logging.Logger:
    logger = logging.getLogger("calibrate_token_alignment")
    logger.setLevel(logging.DEBUG)
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
        logger.addHandler(ch)
    return logger


def main() -> None:
    logger = setup_logging()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    cfg = RobotConfig()
    conn = GCodeConnection(config=cfg)
    cameras = CameraManager(config=cfg)

    logger.info("Oeffne serielle Verbindung und Kameras ...")
    conn.open()
    cameras.open_all()

    try:
        # 1. Ermittle Greif-Position direkt beim Start
        start_pos = conn.get_position()
        x_grasp, y_grasp, z_grasp = start_pos
        logger.info(
            f"Greif-Position ermittelt: X={x_grasp:.2f}, Y={y_grasp:.2f}, Z={z_grasp:.2f}"
        )

        # 2. Schrittweise Z erhoehen, bis Marker erkannt wird
        z_curr = z_grasp
        marker_pixel = None

        logger.info("Starte schrittweise Erhoehung von Z zur Markersuche ...")
        while z_curr < Z_MAX_MM:
            z_curr += Z_STEP_MM
            if z_curr > Z_MAX_MM:
                z_curr = Z_MAX_MM

            logger.info(f"Fahre auf Z={z_curr:.1f} mm ...")
            conn.move(x_grasp, y_grasp, z_curr, cfg.feed_vertical)
            time.sleep(cfg.camera_settle_time)

            frame = cameras.capture_hand()
            if frame is not None:
                p = detect_marker_0(frame)
                if p is not None:
                    logger.info(f"Marker erkannt bei Z={z_curr:.1f} mm an Pixel: {p}")
                    marker_pixel = p
                    break
            else:
                logger.warning(f"Kein Bild von Handkamera bei Z={z_curr:.1f} mm empfangen.")

        if marker_pixel is None:
            raise RuntimeError(
                f"Marker ID 0 konnte bis zur maximalen Hoehe von Z={Z_MAX_MM:.1f} mm nicht gefunden werden."
            )

        # 3. Zusaetzlichen Offset fuer optimale Lesehoehe anfahren
        z_read = z_curr + ADDITIONAL_HEIGHT_MM
        if z_read > Z_MAX_MM:
            z_read = Z_MAX_MM
            logger.warning(
                f"Zusaetzlicher Offset begrenzt auf Z_MAX ({Z_MAX_MM:.1f} mm)."
            )

        logger.info(f"Fahre auf optimale Lesehoehe Z={z_read:.1f} mm ...")
        conn.move(x_grasp, y_grasp, z_read, cfg.feed_vertical)
        time.sleep(cfg.camera_settle_time)

        # 4. Endgueltige Pixel-Koordinaten an Lesehoehe ermitteln
        final_frame = cameras.capture_hand()
        if final_frame is None:
            raise RuntimeError("Kein Bild von Handkamera auf Lesehoehe empfangen.")

        final_pixel = detect_marker_0(final_frame)
        if final_pixel is None:
            raise RuntimeError(
                f"Marker ID 0 auf Lesehoehe Z={z_read:.1f} mm verloren."
            )

        logger.info(
            f"Optimale Erkennungsposition bestimmt: Z={z_read:.1f} mm -> Pixel {final_pixel}"
        )

        # 5. Speichern der Konfiguration
        alignment_data = {
            "grasp_position_xyz": [x_grasp, y_grasp, z_grasp],
            "reading_height_z": z_read,
            "recognition_pixel_xy": list(final_pixel),
        }

        with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
            json.dump(alignment_data, f, indent=4)

        logger.info(f"Kalibrierungsdaten erfolgreich gespeichert unter {OUTPUT_PATH}")

    except Exception as e:
        logger.error(f"Fehler waehrend des Kalibrierungsprozesses: {e}")
        raise
    finally:
        logger.info("Schliesse Verbindungen ...")
        try:
            cameras.close_all()
            conn.close()
        except Exception as exc:
            logger.warning(f"Fehler beim Schliessen der Verbindungen: {exc}")


if __name__ == "__main__":
    main()
