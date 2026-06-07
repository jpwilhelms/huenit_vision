#!/usr/bin/env python3
"""
map_overhead_workspace.py

Faehrt ein feinmaschiges Gitter auf der kalibrierten Tischhoehe Z ab
und verifiziert fuer jeden Punkt (Soll/Ist), ob er physisch erreichbar ist.
Gibt eine ASCII-Karte des Arbeitsraums aus und speichert die Ergebnisse in JSON.
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
from huenit_vision.workspace import check_reachable

OUTPUT_DIR = os.path.join(PROJECT_ROOT, "calib_results")
ALIGNMENT_PATH = os.path.join(OUTPUT_DIR, "token_alignment.json")
MAP_OUTPUT_PATH = os.path.join(OUTPUT_DIR, "overhead_workspace_map.json")

# Gitter-Bereiche (erweiterter Scan)
X_START, X_END, X_STEP = -150.0, 150.0, 25.0
Y_START, Y_END, Y_STEP = 150.0, 350.0, 25.0


def setup_logging() -> logging.Logger:
    logger = logging.getLogger("map_overhead_workspace")
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        ch.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
        logger.addHandler(ch)
    return logger


def main() -> None:
    logger = setup_logging()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 1. Z-Arbeitshoehe aus token_alignment.json laden
    z_grasp = -42.5
    if os.path.exists(ALIGNMENT_PATH):
        try:
            with open(ALIGNMENT_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
                z_grasp = data["grasp_position_xyz"][2]
                logger.info(f"Kalibrierte Greifhoehe geladen: Z={z_grasp:.2f} mm")
        except Exception as e:
            logger.warning(f"Fehler beim Laden von token_alignment.json ({e}). Verwende Default Z={z_grasp}")
    else:
        logger.warning(f"Keine token_alignment.json gefunden. Verwende Default Z={z_grasp}")

    cfg = RobotConfig()
    conn = GCodeConnection(config=cfg)

    logger.info("Oeffne Verbindung zum Roboter ...")
    conn.open()

    # Koordinaten-Listen erzeugen (Y absteigend, damit die ASCII-Karte oben Norden/großes Y anzeigt)
    x_coords = []
    curr_x = X_START
    while curr_x <= X_END + 0.01:
        x_coords.append(round(curr_x, 1))
        curr_x += X_STEP

    y_coords = []
    curr_y = Y_END
    while curr_y >= Y_START - 0.01:
        y_coords.append(round(curr_y, 1))
        curr_y -= Y_STEP

    z_travel = z_grasp + cfg.z_travel_offset
    results = {}
    ascii_grid = {y: {x: "." for x in x_coords} for y in y_coords}

    def verify_reached(tx: float, ty: float, tz: float, tol: float = 1.5) -> bool:
        try:
            act_x, act_y, act_z = conn.get_position()
            dx = abs(act_x - tx)
            dy = abs(act_y - ty)
            dz = abs(act_z - tz)
            if dx > tol or dy > tol or dz > tol:
                return False
            return True
        except Exception:
            return False

    try:
        logger.info("Starte Workspace-Mapping...")
        logger.info(f"Fahre zunaechst auf Sicherheitshoehe Z={z_travel:.2f} mm")
        
        # Startposition einlesen und anheben
        pos = conn.get_position()
        conn.move(pos[0], pos[1], z_travel, cfg.feed_travel)
        time.sleep(1.0)

        for y in y_coords:
            for x in x_coords:
                # 1. Vorab-Software-Check
                if not check_reachable(x, y, z_grasp):
                    results[f"{x},{y}"] = {
                        "status": "software_unreachable",
                        "x": x, "y": y, "z": z_grasp
                    }
                    ascii_grid[y][x] = "."
                    continue

                logger.info(f"Pruefe Punkt: X={x:.1f}, Y={y:.1f}, Z={z_grasp:.1f}...")

                # 2. Fahrt auf Sicherheitshoehe ueber dem Punkt
                conn.move(x, y, z_travel, cfg.feed_travel)
                if not verify_reached(x, y, z_travel):
                    logger.warning(f"  -> Reisesicherheitshoehe nicht erreicht fuer X={x:.1f}, Y={y:.1f}")
                    results[f"{x},{y}"] = {
                        "status": "travel_unreachable",
                        "x": x, "y": y, "z": z_grasp
                    }
                    ascii_grid[y][x] = "X"
                    continue

                # 3. Absenken auf Arbeitshoehe
                conn.move(x, y, z_grasp, cfg.feed_vertical)
                time.sleep(0.3)

                # 4. Ist-Soll-Vergleich durchfuehren
                if verify_reached(x, y, z_grasp):
                    logger.info("  -> [OK] Punkt physisch erreichbar.")
                    results[f"{x},{y}"] = {
                        "status": "reachable",
                        "x": x, "y": y, "z": z_grasp
                    }
                    ascii_grid[y][x] = "O"
                else:
                    try:
                        act = conn.get_position()
                        logger.warning(
                            f"  -> [FEHLER] Abweichung zu gross! Soll: ({x:.1f}, {y:.1f}, {z_grasp:.2f}) "
                            f"Ist: ({act[0]:.2f}, {act[1]:.2f}, {act[2]:.2f})"
                        )
                        results[f"{x},{y}"] = {
                            "status": "physical_unreachable",
                            "x": x, "y": y, "z": z_grasp,
                            "actual": list(act)
                        }
                    except Exception:
                        results[f"{x},{y}"] = {
                            "status": "physical_unreachable",
                            "x": x, "y": y, "z": z_grasp,
                            "actual": None
                        }
                    ascii_grid[y][x] = "X"

                # 5. Zurueck auf Sicherheitshoehe
                conn.move(x, y, z_travel, cfg.feed_vertical)

        # 6. Ergebnisse sichern
        map_data = {
            "z_height": z_grasp,
            "x_range": [X_START, X_END, X_STEP],
            "y_range": [Y_START, Y_END, Y_STEP],
            "points": results
        }
        with open(MAP_OUTPUT_PATH, "w", encoding="utf-8") as f:
            json.dump(map_data, f, indent=4)
        logger.info(f"Workspace-Map erfolgreich gespeichert unter {MAP_OUTPUT_PATH}")

        # 7. ASCII-Karte ausgeben
        print("\n" + "=" * 50)
        print(f"PHYSIKALISCHE ARBEITSBEREICH-KARTE BEI Z = {z_grasp:.2f} mm")
        print("=" * 50)
        print("Legende: O = Erreichbar, X = Blockiert/Fehler, . = Software-Limit\n")
        
        # Spaltenbeschriftung (X)
        header = "      " + " ".join(f"{int(x):>4}" for x in x_coords)
        print(header)
        print("      " + "-" * (len(header) - 6))

        for y in y_coords:
            row_str = " ".join(f"{ascii_grid[y][x]:>4}" for x in x_coords)
            print(f"Y={int(y):<3} | {row_str}")

        print("=" * 50)

    except Exception as e:
        logger.error(f"Schwerer Fehler waehrend des Workspace-Mappings: {e}")
        raise
    finally:
        logger.info("Schliesse Verbindungen ...")
        try:
            conn.close()
        except Exception as exc:
            logger.warning(f"Fehler beim Schliessen: {exc}")


if __name__ == "__main__":
    main()
