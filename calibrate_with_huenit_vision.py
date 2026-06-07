#!/usr/bin/env python3
"""
calibrate_with_huenit_vision.py

Zweistufige Kalibrierung:
Phase 1:
- Ermittlung der initialen Greifposition (Greif-Pos) inkl. Spalt
- Schrittweises Anheben von Z zur Markersuche über die Handkamera (Lesehoehe)
- Initialer Z-Pick-Suchlauf abwaerts zur Bestimmung der spaltbereinigten Greifhoehe
- Speichern der exakten Start-Koordinaten in token_alignment.json

Phase 2:
- 18-Punkt-X/Y-Kalibrierung für die Deckenkamera
- Abfahren des Rasters im Arbeitsbereich
- Handkamera-Visual-Servoing vor jedem Pick zur Ausrichtung des Greifers über dem Token
- Abbruch, falls die Feinjustierung nicht innerhalb der Toleranz konvergiert
- Absichern jedes Picks durch Ueberpruefung der Handkamera (Marker darf nach Anheben auf Lesehoehe nicht mehr sichtbar sein)
- Dynamischer Z-Pick-Suchlauf (1mm Schritte abwaerts) zur Tischhoehenbestimmung an jedem Gitterpunkt
- Speichern der korrigierten X/Y/Z-Werte in der Gitter-Struktur
- Berechnen und Speichern der affinen Transformationsmatrix als overhead_transform.json
"""

import os
import sys
import time
import json
import logging
import dataclasses

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
PROGRESS_PATH = os.path.join(OUTPUT_DIR, "calibration_progress.json")


def save_progress(data: dict) -> None:
    with open(PROGRESS_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4)


def load_progress() -> dict:
    if os.path.exists(PROGRESS_PATH):
        try:
            with open(PROGRESS_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return None


def delete_progress() -> None:
    if os.path.exists(PROGRESS_PATH):
        try:
            os.remove(PROGRESS_PATH)
        except Exception:
            pass

# Parameter fuer Phase 1
Z_STEP_MM = 5.0
Z_MAX_MM = 160.0
ADDITIONAL_HEIGHT_MM = 20.0

# 18 Gitterpunkte im Arbeitsbereich (X: -125 bis 125, Y: 200 bis 300)
CALIBRATION_POINTS = [
    # Y = 200.0 (Reihe 1 nahe der Basis - voll erreichbar)
    (-125.0, 200.0), (-75.0, 200.0), (-25.0, 200.0), (25.0, 200.0), (75.0, 200.0), (125.0, 200.0),
    # Y = 250.0 (Reihe 2 Mitte - voll erreichbar)
    (-125.0, 250.0), (-75.0, 250.0), (-25.0, 250.0), (25.0, 250.0), (75.0, 250.0), (125.0, 250.0),
    # Y = 300.0 (Reihe 3 Vorne - voll erreichbar)
    (-125.0, 300.0), (-75.0, 300.0), (-25.0, 300.0), (25.0, 300.0), (75.0, 300.0), (125.0, 300.0),
]


def setup_logging() -> logging.Logger:
    logging.basicConfig(
        level=logging.DEBUG,
        format="[%(levelname)s] %(name)s: %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    return logging.getLogger("calibrate_huenit_vision")


def main() -> None:
    logger = setup_logging()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Korrigierte Vorzeichen der Visual-Servoing-Gains zur Vermeidung von Divergenz
    cfg = RobotConfig(servoing_gain_x=0.12, servoing_gain_y=-0.12)
    conn = GCodeConnection(config=cfg)
    cameras = CameraManager(config=cfg)

    logger.info("Oeffne serielle Verbindung und Kameras ...")
    conn.open()
    cameras.open_all()

    try:
        progress_data = load_progress()
        resuming = progress_data is not None

        if resuming:
            logger.info("--- FORTSETZEN DER KALIBRIERUNG (Resume-Modus) ---")
            if os.path.exists(ALIGNMENT_PATH):
                with open(ALIGNMENT_PATH, "r", encoding="utf-8") as f:
                    align_info = json.load(f)
                x_grasp, y_grasp, _ = align_info["grasp_position_xyz"]
            else:
                x_grasp, y_grasp = -9.2, 257.06
            
            z_grasp = progress_data["z_grasp"]
            z_start_success = progress_data["z_start_success"]
            z_read = progress_data["z_read"]
            final_pixel = progress_data["final_pixel"]
            grid_data = progress_data["grid_data"]
            next_point_idx = progress_data["next_point_idx"]
            z_ref_base = z_start_success
            logger.info(f"Fortschritt geladen. Lesehoehe Z={z_read:.1f} mm, Start Z-Grasp: {z_start_success:.2f} mm")
        else:
            # ==========================================
            # PHASE 1a: Start-Position und Lesehöhe bestimmen
            # ==========================================
            logger.info("--- START PHASE 1a: Z-Suche & Lesehoehe bestimmen ---")
            start_pos = conn.get_position()
            x_grasp, y_grasp, z_grasp = start_pos
            logger.info(
                f"Initiale Greifer-Position (mit evtl. Spalt): X={x_grasp:.2f}, Y={y_grasp:.2f}, Z={z_grasp:.2f}"
            )

            z_curr = z_grasp
            marker_pixel = None

            logger.info("Suche Marker ueber Handkamera durch schrittweises Anheben...")
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
                    logger.warning(f"Kein Bild von Handkamera bei Z={z_curr:.1f} mm.")

            if marker_pixel is None:
                raise RuntimeError(
                    f"Marker ID 0 konnte bis Z={Z_MAX_MM:.1f} mm nicht detektiert werden."
                )

            z_read = z_curr + ADDITIONAL_HEIGHT_MM
            if z_read > Z_MAX_MM:
                z_read = Z_MAX_MM

            logger.info(f"Fahre auf Lesehoehe Z={z_read:.1f} mm ...")
            conn.move(x_grasp, y_grasp, z_read, cfg.feed_vertical)
            time.sleep(cfg.camera_settle_time)

            final_frame = cameras.capture_hand()
            if final_frame is None:
                raise RuntimeError("Kein Bild von Handkamera auf Lesehoehe empfangen.")

            final_pixel = detect_marker_0(final_frame)
            if final_pixel is None:
                raise RuntimeError(
                    f"Marker auf Lesehoehe Z={z_read:.1f} mm verloren."
                )

        logger.info(
            f"Optimale Erkennungsposition bestimmt: Z={z_read:.1f} mm -> Pixel {final_pixel}"
        )

        # ==========================================
        # Hilfsfunktionen fuer Positionsverifikation, Pick und Place
        # ==========================================
        z_travel = z_grasp + cfg.z_travel_offset
        z_place_default = z_grasp + cfg.z_place_offset
        z_ref_base = z_start_success if resuming else z_grasp

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
                        f"Ist: ({act_x:.2f}, {act_y:.2f}, {act_z:.2f}), Diff: ({dx:.2f}, {dy:.2f}, {dz:.2f})"
                    )
                    return False
                return True
            except Exception as e:
                logger.error(f"Fehler bei Positionsverifikation: {e}")
                return False

        # Dynamische Z-Suche (1mm Schritte abwaerts) mit Ist-Soll-Kontrolle
        def perform_pick(cx: float, cy: float) -> float:
            nonlocal z_ref_base
            logger.info(f"Starte Z-Suche fuer Pick bei X={cx:.1f}, Y={cy:.1f}...")
            z_try = z_ref_base
            
            while z_try >= (z_ref_base - 5.05):
                logger.info(f"Fahre fuer Pick auf Z={z_try:.2f} mm...")
                conn.move(cx, cy, z_travel, cfg.feed_travel)
                if not verify_reached(cx, cy, z_travel):
                    raise RuntimeError(f"Zielposition X={cx:.1f}, Y={cy:.1f} auf Reiseschnittstelle nicht erreicht.")
                
                conn.move(cx, cy, z_try, cfg.feed_vertical)
                if not verify_reached(cx, cy, z_try):
                    logger.warning(f"Z-Zielhoehe {z_try:.2f} mm nicht erreichbar. Breche Z-Suche ab.")
                    conn.vacuum_off(blow_off=False)
                    time.sleep(cfg.vacuum_release_time)
                    break
                
                # Vakuum aufbauen (0,5s)
                conn.vacuum_on()
                time.sleep(cfg.vacuum_buildup_time)
                
                # Auf Lesehoehe anheben zur optischen Verifikation
                conn.move(cx, cy, z_read, cfg.feed_vertical)
                time.sleep(cfg.camera_settle_time)
                
                # Handkamera-Check (Marker darf nicht mehr sichtbar sein)
                test_frame = cameras.capture_hand()
                if test_frame is not None:
                    p = detect_marker_0(test_frame)
                    if p is None:
                        logger.info(f"Pick erfolgreich verifiziert bei Z={z_try:.2f} mm.")
                        return z_try
                    else:
                        logger.warning(f"Marker noch sichtbar auf Tisch bei Z={z_try:.2f} mm.")
                else:
                    logger.warning("Kein Bild zur Pick-Verifikation verfuegbar.")
                
                # Vakuum loesen fuer naechsten Suchschritt
                conn.vacuum_off(blow_off=False)
                time.sleep(cfg.vacuum_release_time)
                
                # 1mm tiefer fahren
                z_try -= 1.0
                
            raise RuntimeError(
                f"Pick fehlgeschlagen. Token konnte bis max. Tiefe bei X={cx:.1f}, Y={cy:.1f} nicht gegriffen werden."
            )

        def perform_place(tx: float, ty: float, z_target: float) -> bool:
            logger.info(f"Platziere Token bei X={tx:.1f}, Y={ty:.1f}, Z={z_target:.2f}...")
            conn.move(tx, ty, z_travel, cfg.feed_travel)
            if not verify_reached(tx, ty, z_travel):
                logger.warning(f"Reisehoehe ueber Platzierungspunkt ({tx:.1f}, {ty:.1f}) nicht erreichbar.")
                return False
                
            conn.move(tx, ty, z_target, cfg.feed_vertical)
            if not verify_reached(tx, ty, z_target):
                logger.warning(f"Ablagehoehe Z={z_target:.2f} bei ({tx:.1f}, {ty:.1f}) nicht erreichbar.")
                # Zur Sicherheit wieder hochfahren, ohne das Vakuum freizugeben
                conn.move(tx, ty, z_travel, cfg.feed_vertical)
                return False
            
            # Vakuum freigeben mit Blow-Off (1,5s)
            conn.vacuum_off(blow_off=True)
            time.sleep(cfg.vacuum_release_time)
            
            # Auf Sicherheitshoehe anheben
            conn.move(tx, ty, z_travel, cfg.feed_vertical)
            return True

        # ==========================================
        # PHASE 1b: Initialer Pick & Config/Servoing initialisieren
        # ==========================================
        if not resuming:
            logger.info("--- START PHASE 1b: Initialer Z-Pick & Alignment sichern ---")
            z_start_success = perform_pick(x_grasp, y_grasp)
            z_ref_base = z_start_success  # Z-Referenz auf reale Tischhoehe setzen
            
            # Alignment spaltbereinigt sichern
            alignment_data = {
                "grasp_position_xyz": [x_grasp, y_grasp, z_start_success],
                "reading_height_z": z_read,
                "recognition_pixel_xy": list(final_pixel),
            }
            with open(ALIGNMENT_PATH, "w", encoding="utf-8") as f:
                json.dump(alignment_data, f, indent=4)
            logger.info(f"Spaltbereinigte Alignment-Daten gespeichert unter {ALIGNMENT_PATH}")

            # Fortschrittsdatei initial anlegen
            progress_data = {
                "next_point_idx": 0,
                "token_on_table": False,
                "current_token_pos": [x_grasp, y_grasp],
                "z_grasp": z_grasp,
                "z_start_success": z_start_success,
                "z_read": z_read,
                "final_pixel": list(final_pixel),
                "grid_data": {}
            }
            save_progress(progress_data)

        # Config kopieren und z_verify dynamisch setzen fuer Visual Servoing
        cfg = dataclasses.replace(cfg, z_verify=z_read)
        servoing = HandCameraServoing(conn, cameras, cfg)
        servoing._target_pixel = final_pixel

        # Resume-spezifisches Aufheben des Tokens vom Tisch
        if resuming:
            token_on_table = progress_data.get("token_on_table", False)
            current_token_pos = progress_data.get("current_token_pos", [x_grasp, y_grasp])
            if token_on_table:
                logger.info(
                    f"Token liegt laut Fortschrittsdatei auf dem Tisch bei X={current_token_pos[0]:.2f}, "
                    f"Y={current_token_pos[1]:.2f}. Greife Token vor Fortsetzung..."
                )
                z_success = perform_pick(current_token_pos[0], current_token_pos[1])
                logger.info(f"Token erfolgreich gegriffen bei Z={z_success:.2f} mm.")
                progress_data["token_on_table"] = False
                save_progress(progress_data)
            else:
                logger.info("Token befindet sich bereits am Greifer.")

        # ==========================================
        # PHASE 2: Deckenkamera X/Y-Kalibrierung (18 Punkte)
        # ==========================================
        logger.info("--- START PHASE 2: Deckenkamera X/Y-Kalibrierung ---")
        
        current_token_x = x_grasp
        current_token_y = y_grasp
        valid_points_count = 0
        if resuming:
            valid_points_count = len(grid_data)

        # Die Schleife startet. Das Token ist bereits am Greifer fixiert.
        for idx, (cal_x, cal_y) in enumerate(CALIBRATION_POINTS):
            pt_idx = idx + 1
            if resuming and idx < next_point_idx:
                logger.info(f"[Gitterpunkt {pt_idx}/{len(CALIBRATION_POINTS)}] Bereits abgeschlossen. Überspringe.")
                continue

            logger.info(
                f"[Gitterpunkt {pt_idx}/{len(CALIBRATION_POINTS)}] Ziel: X={cal_x:.1f}, Y={cal_y:.1f}"
            )
            
            # Sicherheitscheck
            if not check_reachable(cal_x, cal_y, z_grasp):
                logger.warning(f"Zielpunkt ({cal_x:.1f}, {cal_y:.1f}) nicht erreichbar laut Kinematik-Modul. Überspringe.")
                continue
            
            # 1. Ablegen am Zielpunkt
            if not perform_place(cal_x, cal_y, z_place_default):
                logger.warning(f"Gitterpunkt {pt_idx} physisch nicht erreichbar. Überspringe Punkt.")
                continue
            
            # Fortschritt speichern: Token liegt auf dem Tisch
            progress_data = {
                "next_point_idx": idx,
                "token_on_table": True,
                "current_token_pos": [cal_x, cal_y],
                "z_grasp": z_grasp,
                "z_start_success": z_start_success,
                "z_read": z_read,
                "final_pixel": list(final_pixel),
                "grid_data": grid_data
            }
            save_progress(progress_data)
            
            # 2. Arm in Parkposition bewegen, um Sichtfeld fuer Deckenkamera freizumachen
            conn.move(cfg.park_x, cfg.park_y, z_travel, cfg.feed_travel)
            if not verify_reached(cfg.park_x, cfg.park_y, z_travel):
                logger.error("Parkposition konnte nicht angefahren werden! Breche Kalibrierung ab.")
                raise RuntimeError("Parkposition nicht erreichbar.")
                
            time.sleep(cfg.camera_settle_time)
            
            # 3. Deckenkamera-Bild aufnehmen und Marker detektieren
            p_cam = None
            overhead_frame = cameras.capture_overhead()
            if overhead_frame is not None:
                p_cam = detect_marker_0(overhead_frame)
            
            # 4. Vor dem Greifen: Feinjustierung (Visual Servoing) ueber Handkamera ausfuehren
            logger.info(f"Fahre auf Lesehoehe Z={z_read:.1f} mm zur Feinjustierung...")
            adjusted = servoing.fine_adjust(cal_x, cal_y)
            if adjusted is None:
                raise RuntimeError("Feinjustierung abgebrochen: Marker verloren oder Arbeitsraum verlassen.")
                
            x_adj, y_adj = adjusted
            
            # Verifizieren, ob die Feinjustierung wirklich innerhalb der Toleranz konvergiert ist
            time.sleep(cfg.camera_settle_time)
            verify_frame = cameras.capture_hand()
            if verify_frame is None:
                raise RuntimeError("Kein Bild zur Verifikation der Feinjustierung verfuegbar.")
            p_verify = detect_marker_0(verify_frame)
            if p_verify is None:
                raise RuntimeError("Marker nach Feinjustierung verloren.")
                
            err_u = p_verify[0] - final_pixel[0]
            err_v = p_verify[1] - final_pixel[1]
            dist_px = (err_u**2 + err_v**2)**0.5
            
            # Verifikationstoleranz gelockert auf 8.0 px
            if dist_px > 8.0:
                raise RuntimeError(
                    f"Feinjustierung nicht konvergiert! Endfehler: {dist_px:.1f}px (Toleranz: 8.0px)"
                )
                
            logger.info(f"Feinjustierung erfolgreich konvergiert: Soll ({cal_x:.1f}, {cal_y:.1f}) -> Ist ({x_adj:.2f}, {y_adj:.2f})")
            
            # Aktuelle Ist-Position des Tokens auf dem Tisch speichern
            progress_data["current_token_pos"] = [x_adj, y_adj]
            save_progress(progress_data)
            
            # 5. Detektions-Daten temporaer speichern (wir nutzen die tatsaechliche Ist-Position x_adj, y_adj!)
            if p_cam is not None:
                logger.info(f"Deckenkamera-Detektion erfolgreich: Robot ({x_adj:.2f}, {y_adj:.2f}) -> Pixel {p_cam}")
                grid_data[f"point_{idx}"] = {
                    "robot": [x_adj, y_adj],
                    "pixel": list(p_cam),
                }
            else:
                logger.warning(f"Marker an Gitterpunkt {pt_idx} über Deckenkamera nicht detektiert.")
            
            # 6. Token wieder aufnehmen und reale Z-Hoehe bestimmen
            try:
                z_success = perform_pick(x_adj, y_adj)
                if p_cam is not None:
                    grid_data[f"point_{idx}"]["robot_z"] = z_success
                    valid_points_count += 1
            except Exception as e:
                logger.error(f"Fehler beim Aufnehmen des Tokens von Punkt {pt_idx}: {e}")
                raise
            
            current_token_x = x_adj
            current_token_y = y_adj

            # Fortschritt speichern: Punkt ist komplett abgeschlossen, Token ist am Greifer
            progress_data = {
                "next_point_idx": idx + 1,
                "token_on_table": False,
                "current_token_pos": [x_adj, y_adj],
                "z_grasp": z_grasp,
                "z_start_success": z_start_success,
                "z_read": z_read,
                "final_pixel": list(final_pixel),
                "grid_data": grid_data
            }
            save_progress(progress_data)
            
        # Fortschrittsdatei nach erfolgreichem Durchlauf loeschen
        delete_progress()
        
        # Token zum Abschluss an die Startposition zurueckbringen
        if valid_points_count > 0:
            logger.info("Kalibrierung beendet. Bringe Token zurück zur Startposition...")
            try:
                # Token ist bereits am Greifer fixiert. Direkt ablegen.
                perform_place(x_grasp, y_grasp, z_start_success + cfg.z_place_offset)
            except Exception as e:
                logger.error(f"Fehler beim Zurueckbringen des Tokens zur Startposition: {e}")

        # 7. Transformationsmatrix berechnen und speichern
        if len(grid_data) >= 3:
            logger.info(f"Berechne affine Transformation aus {len(grid_data)} gueltigen Messpunkten...")
            transform = CoordinateTransform()
            transform.fit_from_grid_data(grid_data)
            transform.save(TRANSFORM_PATH)
            logger.info(f"Transformationsmatrizen erfolgreich unter {TRANSFORM_PATH} gespeichert.")
        else:
            logger.error("Zu wenige gueltige Punkte zur Berechnung der Transformation. Mindestens 3 erforderlich.")

    except Exception as e:
        logger.error(f"Kritischer Fehler waehrend der Kalibrierung: {e}")
        raise
    finally:
        logger.info("Schliesse Verbindungen und schalte Vakuum ab...")
        try:
            conn.vacuum_off(blow_off=False)
            cameras.close_all()
            conn.close()
        except Exception as exc:
            logger.warning(f"Fehler beim Aufraeumen: {exc}")


if __name__ == "__main__":
    main()
