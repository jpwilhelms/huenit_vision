import os
import json
import time
import logging
from typing import Dict, Tuple, Any

from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.calibration.hand_camera_servoing import HandCameraServoing
from huenit_vision.config import RobotConfig

logger = logging.getLogger(__name__)

class GridMapper:
    """BFS‑based grid calibration.

    The implementation follows the specification in ``huenit_vision_spec.md``.
    It iteratively explores a square lattice around the start point, places a
    reference object, captures the overhead image, and records the pixel
    coordinates of the ArUco marker.
    """

    def __init__(self, conn: GCodeConnection, cameras: CameraManager,
                 servoing: HandCameraServoing, config: RobotConfig):
        self.conn = conn
        self.cameras = cameras
        self.servoing = servoing
        self.config = config
        self.grid_data: Dict[str, Dict[str, Any]] = {}

    def _log(self, msg: str) -> None:
        logger.info(msg)

    def _save_incremental(self, output_path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(self.grid_data, f, indent=4)
        self._log(f"Incremental calibration data written to {output_path}")

    def run(self, x_start: float, y_start: float, z_start: float,
            output_path: str = "./calib_results/aruco_grid_mapping.json") -> Dict[str, Any]:
        """Execute the full calibration routine.

        Returns the accumulated ``grid_data`` dictionary.
        """
        # Phase 1 – initialization
        Z_place = self.config.get_z_place(z_start)
        Z_pick = self.config.get_z_pick(z_start)
        Z_travel = self.config.get_z_travel(z_start)

        self.conn.home()
        # reference pixel centre via hand camera servoing
        U_target, V_target = self.servoing.calibrate_reference(x_start, y_start)
        self._log(f"Hand‑camera reference pixel: ({U_target}, {V_target})")

        # park and capture the overhead origin pixel (u0, v0)
        self.conn.move(self.config.park_x, self.config.park_y, Z_travel,
                       self.config.feed_travel)
        time.sleep(self.config.camera_settle_time)
        frame = self.cameras.capture_overhead()
        origin_pixel = None
        if frame is not None:
            from huenit_vision.aruco import detect_marker_0
            origin_pixel = detect_marker_0(frame)
        if origin_pixel is None:
            raise RuntimeError("Failed to capture origin marker during calibration")
        u0, v0 = origin_pixel
        self.grid_data["0,0"] = {"robot": [x_start, y_start], "pixel": [u0, v0]}
        self._save_incremental(output_path)

        # Phase 2 – initial pick (required to have an object on the table)
        from huenit_vision.operations.pick import pick_object
        if not pick_object(self.conn, self.cameras, self.servoing,
                           x_start, y_start, Z_pick, Z_travel, self.config):
            raise RuntimeError("Initial pick failed – cannot start grid exploration")

        # Phase 3 – BFS exploration
        from collections import deque
        step = self.config.grid_step_mm
        queue = deque([(1, 0), (-1, 0), (0, 1), (0, -1)])
        visited = {(0, 0)}
        while queue:
            i, j = queue.popleft()
            if (i, j) in visited:
                continue
            visited.add((i, j))
            rx = x_start + i * step
            ry = y_start + j * step
            # workspace check
            if not (self.conn.move_and_verify(rx, ry, Z_travel, self.config.feed_travel) and
                    self.conn.move_and_verify(rx, ry, Z_place, self.config.feed_vertical)):
                self._log(f"Skipping unreachable grid point {(i, j)}")
                continue
            # place object
            self.conn.vacuum_off(blow_off=True)
            self.conn.move(rx, ry, Z_travel, self.config.feed_vertical)

            # measurement
            self.conn.move(self.config.park_x, self.config.park_y, Z_travel,
                           self.config.feed_travel)
            time.sleep(self.config.camera_settle_time)
            frame = self.cameras.capture_overhead()
            if frame is None:
                self._log(f"No frame captured at {(i, j)}")
                continue
            from huenit_vision.aruco import detect_marker_0
            p = detect_marker_0(frame)
            if p is None:
                self._log(f"Marker not visible at {(i, j)} – outside view")
                continue
            # duplicate check – ensure spacing >4px
            last = self.grid_data.get(f"{i},{j}")
            if last:
                prev_px, prev_py = last["pixel"]
                if ((p[0] - prev_px) ** 2 + (p[1] - prev_py) ** 2) ** 0.5 < 4:
                    self._log(f"Duplicate pixel detected at {(i, j)} – aborting")
                    break
            self.grid_data[f"{i},{j}"] = {"robot": [rx, ry], "pixel": [p[0], p[1]]}
            self._save_incremental(output_path)
            # enqueue neighbours
            for di, dj in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nxt = (i + di, j + dj)
                if nxt not in visited:
                    queue.append(nxt)
            # re‑pick for next iteration
            if not pick_object(self.conn, self.cameras, self.servoing,
                               rx, ry, Z_pick, Z_travel, self.config):
                self._log(f"Pick failed at {(i, j)} – aborting exploration")
                break
        # Phase 4 – finalisation (ensured by finally block in caller)
        self._log("Calibration run completed")
        return self.grid_data
