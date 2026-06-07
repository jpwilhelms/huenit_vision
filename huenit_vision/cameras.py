# """
# cameras.py – Camera management for overhead and hand cameras.
# Provides a CameraManager class that opens both cameras, configures them, and captures frames.
# """

import cv2
import time
import logging
from typing import Optional
from huenit_vision.config import RobotConfig

logger = logging.getLogger(__name__)

class CameraManager:
    """Manages the two cameras used by the Huenit vision system.

    The overhead camera provides a wide view of the workspace, while the hand camera
    looks at the gripper. Both cameras are opened once at program start and kept open
    until shutdown to avoid the overhead of USB enumeration.
    """

    def __init__(self, config: RobotConfig):
        self.config = config
        self.caps = {}
        self.opened = False

    def _open_camera(self, cam_id: int | str, width: int, height: int, name: str) -> cv2.VideoCapture:
        """Open a camera with the given device identifier and configure it.

        Parameters
        ----------
        cam_id: int | str
            The device index (e.g. 0, 1) or a V4L2 device path like ``/dev/video0``.
        width, height: int
            Desired resolution.
        name: str
            Human‑readable name for log messages.
        """
        cap = cv2.VideoCapture(cam_id)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open {name} camera (id={cam_id})")
        # Use MJPG for low latency
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # Buffer size is critical: we want the freshest frame.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, self.config.camera_buffer_size)
        # Warm‑up frames (discard) to clear pipeline latency
        for _ in range(self.config.camera_warmup_frames):
            cap.read()
        logger.debug(f"{name} camera opened (id={cam_id}, {width}x{height})")
        return cap

    def open_all(self) -> None:
        """Open both overhead and hand cameras.

        This method is idempotent; calling it when the cameras are already open
        does nothing.
        """
        if self.opened:
            return
        self.caps["overhead"] = self._open_camera(
            self.config.overhead_camera_id,
            self.config.overhead_width,
            self.config.overhead_height,
            "Overhead",
        )
        self.caps["hand"] = self._open_camera(
            self.config.hand_camera_id,
            self.config.hand_width,
            self.config.hand_height,
            "Hand",
        )
        self.opened = True

    def close_all(self) -> None:
        """Release both cameras if they are open."""
        for name, cap in self.caps.items():
            try:
                cap.release()
                logger.debug(f"{name} camera released")
            except Exception as e:
                logger.warning(f"Error releasing {name} camera: {e}")
        self.caps.clear()
        self.opened = False

    def _capture(self, key: str) -> Optional[any]:
        """Capture a single frame from the specified camera.

        The function flushes a configurable number of frames before returning the
        most recent one to minimise latency.
        """
        cap = self.caps.get(key)
        if cap is None or not cap.isOpened():
            logger.error(f"{key} camera not opened when capture requested")
            return None
        # Flush stale frames
        for _ in range(self.config.frame_flush_count):
            cap.read()
        ret, frame = cap.read()
        if not ret:
            logger.error(f"Failed to read frame from {key} camera")
            return None
        return frame

    def capture_overhead(self) -> Optional[any]:
        """Capture a fresh frame from the overhead camera."""
        return self._capture("overhead")

    def capture_hand(self) -> Optional[any]:
        """Capture a fresh frame from the hand camera."""
        return self._capture("hand")

    # Context‑manager support (optional but convenient)
    def __enter__(self):
        self.open_all()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close_all()
        # Do not suppress exceptions
        return False
