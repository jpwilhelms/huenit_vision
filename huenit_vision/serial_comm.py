"""
huenit_vision/serial_comm.py
Serial communication interface for sending G-Code commands to the Huenit robot arm.
"""

import time
import re
import warnings
import logging
import threading
import serial
import serial.tools.list_ports
from typing import Tuple, List, Optional
from huenit_vision.config import RobotConfig

logger = logging.getLogger(__name__)

class GCodeConnection:
    """
    Manages the serial G-Code connection to the Huenit robot arm.
    """
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self._serial: Optional[serial.Serial] = None
        self._port: Optional[str] = None
        self._connected: bool = False
        self._lock = threading.RLock()

    @property
    def port(self) -> Optional[str]:
        """Returns the active serial port."""
        with self._lock:
            return self._port

    def is_connected(self) -> bool:
        """Returns True if the serial connection is active and open."""
        with self._lock:
            return self._connected and self._serial is not None and self._serial.is_open

    def scan_ports(self) -> List[Tuple[str, str, Optional[int], Optional[int], Optional[str], Optional[str]]]:
        """Scans available COM/serial ports on the system."""
        ports = list(serial.tools.list_ports.comports())
        return [
            (p.device, p.description, p.vid, p.pid, p.manufacturer, p.serial_number)
            for p in ports
        ]

    def discover_port(self) -> Optional[str]:
        """Scans ports and does a case-insensitive search for 'FTDI'."""
        ports = self.scan_ports()
        if not ports:
            logger.warning("No serial ports detected.")
            return None

        for port in ports:
            manufacturer = port[4] or ""
            description = port[1] or ""
            if "FTDI" in manufacturer.upper() or "FTDI" in description.upper():
                logger.info(f"Discovered FTDI device: {port[0]} ({description})")
                return port[0]

        fallback = ports[0][0]
        logger.info(f"No FTDI device found. Falling back to the first port: {fallback}")
        return fallback

    def open(self, port: Optional[str] = None) -> None:
        """Thread-safely opens the serial connection."""
        with self._lock:
            if self._connected:
                logger.warning("Already connected. Closing existing connection.")
                self._close_under_lock()

            target_port = port if port is not None else self.discover_port()
            if not target_port:
                raise RuntimeError("Connection failed: No serial port found or specified.")

            try:
                self._serial = serial.Serial(
                    port=target_port,
                    baudrate=self.config.baudrate,
                    timeout=self.config.serial_timeout
                )
                self._serial.rts = False
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                self._port = target_port
                self._connected = True
                logger.info(f"Successfully opened connection on port {target_port}.")
            except Exception as e:
                self._close_under_lock()
                raise RuntimeError(f"Failed to open connection to {target_port}: {e}") from e

    def close(self) -> None:
        """Thread-safely closes the serial connection."""
        with self._lock:
            self._close_under_lock()

    def _close_under_lock(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
                logger.info(f"Closed connection on port {self._port}.")
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")
        self._serial = None
        self._port = None
        self._connected = False

    def _send_and_read_under_lock(self, cmd: str) -> List[str]:
        if self._serial is None:
            raise RuntimeError("Serial connection is not open.")

        try:
            if not cmd.endswith('\n'):
                cmd += '\n'

            self._serial.write(cmd.encode('utf-8'))
            
            response_lines = []
            start_time = time.time()

            while True:
                elapsed = time.time() - start_time
                if elapsed > self.config.gcode_response_timeout:
                    msg = f"Timeout ({self.config.gcode_response_timeout}s) waiting for 'ok' for command: {cmd.strip()}"
                    logger.warning(msg)
                    warnings.warn(msg, RuntimeWarning)
                    break

                line_bytes = self._serial.readline()
                if not line_bytes:
                    if not self.is_connected() or self._serial is None or not self._serial.is_open:
                        raise serial.SerialException("Serial connection disconnected during read.")
                    time.sleep(0.001)
                    continue

                line = line_bytes.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                response_lines.append(line)

                if "ok" in line:
                    break

                if "Unknown" in line or "Error" in line:
                    logger.warning(f"Firmware reported error: {line}")

            return response_lines
        except (serial.SerialException, Exception) as e:
            self.close()
            raise

    def send(self, cmd: str) -> List[str]:
        """Splits commands by newlines, trims whitespace, and sends each individually."""
        with self._lock:
            if not self.is_connected() or self._serial is None:
                raise RuntimeError("Serial connection is not open.")

            lines = [line.strip() for line in cmd.splitlines() if line.strip()]
            accumulated_responses = []
            for line in lines:
                responses = self._send_and_read_under_lock(line)
                accumulated_responses.extend(responses)
                
                # Cumulative Command Timeout / Failure check
                has_ok = any("ok" in r for r in responses)
                has_error = any("Unknown" in r or "Error" in r for r in responses)
                if not has_ok or has_error:
                    logger.warning(f"Aborting remaining commands in multi-line block due to timeout or failure in command: {line}")
                    break
            return accumulated_responses

    def get_position(self) -> Tuple[float, float, float]:
        """Queries the current position using M1008 A3 and returns (X, Y, Z)."""
        response_lines = self.send("M1008 A3")
        
        pattern = re.compile(
            r"X\s*[:\s]\s*([+-]?\d+(?:\.\d+)?)"
            r"\s*Y\s*[:\s]\s*([+-]?\d+(?:\.\d+)?)"
            r"\s*Z\s*[:\s]\s*([+-]?\d+(?:\.\d+)?)", 
            re.IGNORECASE
        )

        for line in response_lines:
            match = pattern.search(line)
            if match:
                try:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    z = float(match.group(3))
                    return (x, y, z)
                except ValueError as e:
                    raise ValueError(f"Failed to parse regex coordinates: {e}") from e

        raise RuntimeError(
            f"Coordinates not found in response: {response_lines}"
        )

    def home(self) -> None:
        """Homes the robot, enables motors, and sets absolute coordinates."""
        with self._lock:
            self.send("M1008 A5")
            time.sleep(1.0)
            self.send("M17")
            self.send("G90")

    def enable_motors(self) -> None:
        """Enables motors."""
        with self._lock:
            self.send("M17")

    def vacuum_on(self) -> None:
        """Turns the vacuum pump on at max power with closed valve."""
        with self._lock:
            self.send("M1401 A0")
            power = self.config.pump_max_power
            self.send(f"M1400 A{power}")

    def vacuum_off(self, blow_off: bool = True) -> None:
        """Turns the vacuum pump off, optionally performing a blow-off."""
        with self._lock:
            self.send("M1400 A0")
            if blow_off:
                self.send("M1401 A1")
                time.sleep(self.config.vacuum_release_time)
                self.send("M1401 A0")

    def move(self, x: float, y: float, z: float, feed: int) -> None:
        """Moves to target coordinates and waits for physical completion."""
        with self._lock:
            self.send(f"G1 X{x} Y{y} Z{z} F{feed}")
            self.send("M400")

    def move_and_verify(self, x: float, y: float, z: float, feed: int) -> bool:
        """Moves to target and verifies final coordinates are within tolerance.
        """
        # move acquires its own lock; get_position also acquires lock internally.
        # No outer lock here to allow interleaved high‑level operations.
        self.move(x, y, z, feed)
        try:
            act_x, act_y, act_z = self.get_position()
        except Exception as e:
            logger.error(f"Failed to get position for verification: {e}")
            return False

        tol = self.config.position_tolerance_mm
        dx = abs(act_x - x)
        dy = abs(act_y - y)
        dz = abs(act_z - z)

        return dx <= tol and dy <= tol and dz <= tol
