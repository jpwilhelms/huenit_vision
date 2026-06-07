#!/usr/bin/env python3
"""
keep_token_in_place.py

Dieses Skript merkt sich beim Start die Ursprungsposition des Markers (ID 0).
In einer Endlosschleife ueberwacht es das Token per Deckenkamera.
Wird das Token manuell versetzt, faehrt der Arm dorthin, feinjustiert sich
mittels Handkamera, nimmt das Token auf und bringt es zur Ursprungsposition zurueck.
Während des Prozesses wird die Position des Tokens kontinuierlich ueberwacht;
sollte das Token waehrend der Annaeherung oder des Picken-Vorgangs entfernt oder
erneut verschoben werden, bricht der Vorgang ab und der Arm faehrt in Parkposition.
"""

import os
import sys
import time
import json
import logging
import dataclasses
import math
from typing import Tuple, Optional

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.aruco import detect_marker_0
from huenit_vision.calibration.coordinate_transform import CoordinateTransform
from huenit_vision.calibration.hand_camera_servoing import HandCameraServoing
from huenit_vision.workspace import check_reachable

OUTPUT_DIR = os.path.join(PROJECT_ROOT, "calib_results")
ALIGNMENT_PATH = os.path.join(OUTPUT_DIR, "token_alignment.json")
TRANSFORM_PATH = os.path.join(OUTPUT_DIR, "overhead_transform.json")


def setup_logging() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(name)s: %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("keep_token_in_place")


def main() -> None:
    logger = setup_logging()

    # Validierung der Kalibrierungsdateien
    if not os.path.exists(ALIGNMENT_PATH):
        logger.error(f"Ausrichtungsdatei {ALIGNMENT_PATH} fehlt. Kalibrierung erforderlich.")
        sys.exit(1)
    if not os.path.exists(TRANSFORM_PATH):
        logger.error(f"Transformationsdatei {TRANSFORM_PATH} fehlt. Kalibrierung erforderlich.")
        sys.exit(1)

    # Lade Kalibrierungsdaten
    with open(ALIGNMENT_PATH, "r", encoding="utf-8") as f:
        align_data = json.load(f)
    x_grasp, y_grasp, z_start_success = align_data["grasp_position_xyz"]
    z_read = align_data["reading_height_z"]
    final_pixel = tuple(align_data["recognition_pixel_xy"])

    transform = CoordinateTransform()
    transform.load(TRANSFORM_PATH)

    cfg = RobotConfig(servoing_gain_x=0.12, servoing_gain_y=-0.12)
    cfg = dataclasses.replace(cfg, z_verify=z_read)

    conn = GCodeConnection(config=cfg)
    cameras = CameraManager(config=cfg)

    logger.info("Verbinde mit Roboter und oeffne Kameras...")
    conn.open()
    cameras.open_all()

    z_travel = z_start_success + cfg.z_travel_offset
    z_place_default = z_start_success + cfg.z_place_offset

    def verify_reached(target_x: float, target_y: float, target_z: float) -> bool:
        try:
            act_x, act_y, act_z = conn.get_position()
            tol = cfg.position_tolerance_mm
            dx = abs(act_x - target_x)
            dy = abs(act_y - target_y)
            dz = abs(act_z - target_z)
            if dx > tol or dy > tol or dz > tol:
                logger.warning(
                    f"Position nicht erreicht! Soll: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f}), "
                    f"Ist: ({act_x:.2f}, {act_y:.2f}, {act_z:.2f})"
                )
                return False
            return True
        except Exception as e:
            logger.error(f"Fehler bei Positionsverifikation: {e}")
            return False

    def perform_pick(cx: float, cy: float) -> float:
        # Dynamische Suche ab Z=-39.0 bis Z=-46.0
        z_ref_base = -39.0
        logger.info(f"Starte Pick-Suche an X={cx:.2f}, Y={cy:.2f}...")
        z_try = z_ref_base
        
        while z_try >= -46.0:
            logger.info(f"Fahre fuer Pick auf Z={z_try:.2f} mm...")
            conn.move(cx, cy, z_travel, cfg.feed_travel)
            if not verify_reached(cx, cy, z_travel):
                raise RuntimeError(f"Reisehoehe ueber Zielpunkt nicht erreicht.")
            
            conn.move(cx, cy, z_try, cfg.feed_vertical)
            if not verify_reached(cx, cy, z_try):
                logger.warning(f"Z-Zielhoehe {z_try:.2f} mm nicht erreichbar. Breche Z-Suche ab.")
                conn.vacuum_off(blow_off=False)
                time.sleep(cfg.vacuum_release_time)
                break
            
            conn.vacuum_on()
            time.sleep(cfg.vacuum_buildup_time)
            
            # Auf Lesehoehe anheben zur optischen Verifikation
            conn.move(cx, cy, z_read, cfg.feed_vertical)
            time.sleep(cfg.camera_settle_time)
            
            # Verifikation: Liegt das Token noch auf dem Tisch?
            test_frame = cameras.capture_hand()
            if test_frame is not None:
                p = detect_marker_0(test_frame)
                if p is None:
                    logger.info(f"Pick erfolgreich bei Z={z_try:.2f} mm verifiziert.")
                    return z_try
                else:
                    logger.warning(f"Marker noch auf Tisch sichtbar bei Z={z_try:.2f} mm (nicht gegriffen).")
            else:
                logger.warning("Kein Bild zur Pick-Verifikation verfuegbar.")
            
            conn.vacuum_off(blow_off=False)
            time.sleep(cfg.vacuum_release_time)
            z_try -= 1.0
            
        raise RuntimeError("Pick fehlgeschlagen: Suchbereich erschoepft oder Token entwendet.")

    def perform_place(tx: float, ty: float, z_target: float) -> bool:
        logger.info(f"Platziere Token bei X={tx:.1f}, Y={ty:.1f}, Z={z_target:.2f}...")
        conn.move(tx, ty, z_travel, cfg.feed_travel)
        if not verify_reached(tx, ty, z_travel):
            logger.warning(f"Reisehoehe ueber Ablagepunkt ({tx:.1f}, {ty:.1f}) nicht erreichbar.")
            return False
            
        conn.move(tx, ty, z_target + cfg.z_place_offset, cfg.feed_vertical)
        if not verify_reached(tx, ty, z_target + cfg.z_place_offset):
            logger.warning(f"Ablagehoehe bei ({tx:.1f}, {ty:.1f}) nicht erreichbar.")
            conn.move(tx, ty, z_travel, cfg.feed_vertical)
            return False
        
        conn.vacuum_off(blow_off=True)
        time.sleep(cfg.vacuum_release_time)
        conn.move(tx, ty, z_travel, cfg.feed_vertical)
        return True

    # Setup Visual Servoing
    servoing = HandCameraServoing(conn, cameras, cfg)
    servoing._target_pixel = final_pixel

    try:
        # 1. Parken und Sichtfeld freigeben
        logger.info("Fahre in Parkposition zur Startpositionserfassung...")
        conn.move(cfg.park_x, cfg.park_y, z_travel, cfg.feed_travel)
        time.sleep(cfg.camera_settle_time)

        # 2. Startposition (Ursprungsposition) des Tokens erfassen
        x_start, y_start = None, None
        logger.info("Warte auf Detektion des Tokens auf dem Tisch...")
        
        while x_start is None:
            frame = cameras.capture_overhead()
            if frame is not None:
                p_cam = detect_marker_0(frame)
                if p_cam is not None:
                    x_start, y_start = transform.pixel_to_robot(p_cam[0], p_cam[1])
                    logger.info(f"Ursprungsposition erfasst: X={x_start:.2f}, Y={y_start:.2f} (Pixel: {p_cam})")
                    break
            time.sleep(1.0)

        # 3. Ueberwachungs-Schleife
        logger.info("Ueberwachung aktiv. Verschiebe das Token, um die Rueckfuehrung zu testen.")
        while True:
            # Stelle sicher, dass der Arm in Parkposition ist
            curr_pos = conn.get_position()
            if abs(curr_pos[0] - cfg.park_x) > 5.0 or abs(curr_pos[1] - cfg.park_y) > 5.0:
                logger.info("Fahre in Parkposition...")
                conn.move(cfg.park_x, cfg.park_y, z_travel, cfg.feed_travel)
                time.sleep(cfg.camera_settle_time)

            # Deckenkamera scannen
            frame = cameras.capture_overhead()
            if frame is not None:
                p_cam = detect_marker_0(frame)
                if p_cam is not None:
                    x_curr, y_curr = transform.pixel_to_robot(p_cam[0], p_cam[1])
                    dist = math.hypot(x_curr - x_start, y_curr - y_start)

                    # Wenn das Token um mehr als 12.0 mm verschoben wurde
                    if dist > 12.0:
                        logger.info(f"Token verschoben! Abweichung: {dist:.1f} mm. Aktuelle Position: X={x_curr:.2f}, Y={y_curr:.2f}")
                        
                        try:
                            # Pruefe Erreichbarkeit
                            if not check_reachable(x_curr, y_curr, z_start_success):
                                logger.warning(f"Position ({x_curr:.1f}, {y_curr:.1f}) liegt ausserhalb des Arbeitsraums.")
                                time.sleep(1.0)
                                continue

                            # Annaeherung auf Lesehoehe zur Feinjustierung
                            logger.info("Fahre zur Feinjustierung ueber das verschobene Token...")
                            adjusted = servoing.fine_adjust(x_curr, y_curr)
                            
                            if adjusted is None:
                                logger.warning("Feinjustierung abgebrochen: Token waehrend der Bewegung entfernt/verschoben.")
                                continue

                            x_adj, y_adj = adjusted

                            # Verifikation vor dem Pick: Liegt das Token noch korrekt?
                            time.sleep(cfg.camera_settle_time)
                            verify_frame = cameras.capture_hand()
                            if verify_frame is None:
                                raise RuntimeError("Kein Bild von Handkamera zur Verifikation.")
                            
                            p_verify = detect_marker_0(verify_frame)
                            if p_verify is None:
                                raise RuntimeError("Token waehrend der Verifikation verschwunden.")

                            err_u = p_verify[0] - final_pixel[0]
                            err_v = p_verify[1] - final_pixel[1]
                            dist_px = math.hypot(err_u, err_v)
                            
                            if dist_px > 8.0:
                                raise RuntimeError(f"Token wurde waehrend der Annaeherung bewegt (Pixelabweichung: {dist_px:.1f}px).")

                            # Token aufnehmen
                            perform_pick(x_adj, y_adj)
                            logger.info("Token gegriffen. Bringe es zur Ursprungsposition zurueck...")

                            # Token ablegen
                            perform_place(x_start, y_start, z_start_success)
                            logger.info("Token wieder an der Ursprungsposition platziert.")

                        except Exception as e:
                            logger.error(f"Fehler bei Rueckfuehrung: {e}")
                            logger.info("Fahre in Sicherheitsposition und schalte Vakuum ab...")
                            conn.vacuum_off(blow_off=False)
                            conn.move(conn.get_position()[0], conn.get_position()[1], z_travel, cfg.feed_vertical)

            time.sleep(0.5)

    except KeyboardInterrupt:
        logger.info("Skript manuell beendet.")
    finally:
        logger.info("Schliesse Verbindungen...")
        conn.vacuum_off(blow_off=False)
        cameras.close_all()
        conn.close()


if __name__ == "__main__":
    main()
