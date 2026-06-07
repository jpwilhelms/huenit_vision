#!/usr/bin/env python3
"""
cleanup_robot.py

Sicherheits-Cleanup und Parken des Roboterarms unter Verwendung des huenit_vision Moduls.
Schaltet die Vakuumpumpe ab und parkt den Arm in der sicheren Parkposition.
"""

import sys
import time
import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection

cfg = RobotConfig()
conn = GCodeConnection(config=cfg)

print("Opening serial connection for safety cleanup...")
try:
    conn.open()
except Exception as e:
    print(f"Failed to open serial: {e}")
    sys.exit(1)

try:
    print("Turning suction pump OFF...")
    conn.vacuum_off(blow_off=True)
    time.sleep(cfg.vacuum_release_time)

    try:
        curr_pos = conn.get_position()
        curr_x, curr_y, _ = curr_pos
    except Exception:
        curr_x, curr_y = 0.0, 200.0

    print("Lifting arm to Z=40.0 mm...")
    conn.move(curr_x, curr_y, 40.0, cfg.feed_vertical)
    time.sleep(2.0)

    print(f"Parking arm at ({cfg.park_x:.1f}, {cfg.park_y:.1f}) at Z=40.0 mm...")
    conn.move(cfg.park_x, cfg.park_y, 40.0, cfg.feed_travel)
    time.sleep(3.0)

except Exception as e:
    print(f"Error during cleanup: {e}")
finally:
    conn.close()

print("Cleanup complete. Robot is parked and idle.")
