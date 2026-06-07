"""
huenit_vision/tests/test_serial_comm_stress.py
Empirical stress tests for GCodeConnection in serial_comm.py.
"""

import time
import pytest
import threading
import warnings
import re
from unittest.mock import MagicMock, patch
import serial
from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection

@pytest.fixture
def mock_config():
    return RobotConfig(
        baudrate=115200,
        serial_timeout=0.05,
        gcode_response_timeout=0.1,
        position_tolerance_mm=1.0,
        vacuum_release_time=0.05
    )

@pytest.fixture
def connection(mock_config):
    return GCodeConnection(mock_config)

# --- 1. Chaotic / Corrupt G-Code Strings ---

@patch("serial.Serial")
def test_send_chaotic_corrupt_strings(mock_serial_cls, connection):
    """
    Test sending corrupt, chaotic, or edge-case G-code inputs.
    """
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        
        # Scenario A: Unicode, control characters, binary payloads
        # We split "G1 X10 \x00\x03\r\n\x01\x02 \U0001F600" by lines.
        # It has 2 lines: "G1 X10 \x00\x03" and "\x01\x02 \U0001F600".
        # Therefore we need 2 ok responses.
        mock_serial_instance.readline.side_effect = [b"ok\n", b"ok\n"]
        chaotic_cmd = "G1 X10 \x00\x03\r\n\x01\x02 \U0001F600"
        responses = connection.send(chaotic_cmd)
        # Verify it encoded to utf-8
        encoded_calls = [call[0][0] for call in mock_serial_instance.write.call_args_list]
        assert len(encoded_calls) == 2
        assert any(b"G1 X10" in call for call in encoded_calls)
        assert responses == ["ok", "ok"]

        mock_serial_instance.write.reset_mock()

        # Scenario B: Extremely long G-code line (buffer overflow threat)
        mock_serial_instance.readline.side_effect = [b"ok\n"]
        long_cmd = "G1 " + "X" * 5000
        responses = connection.send(long_cmd)
        mock_serial_instance.write.assert_called_once()
        assert len(mock_serial_instance.write.call_args[0][0]) > 5000
        assert responses == ["ok"]

        mock_serial_instance.write.reset_mock()

        # Scenario C: Empty/Whitespace command
        responses = connection.send("   \n\n   \n")
        # Should not write anything to the serial interface
        mock_serial_instance.write.assert_not_called()
        assert responses == []

# --- 2. Regex Parsing Behavior under Extreme Spacing / Non-numeric / Multi-line ---

@patch("serial.Serial")
def test_regex_parsing_variations(mock_serial_cls, connection):
    """
    Verifies regex coordinate parsing under extreme spacing, extra characters, and missing fields.
    """
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()

        # Scenario A: Tab and extreme whitespace spacing
        mock_serial_instance.readline.side_effect = [
            b"X \t  :\t  10.5 \t Y \t  :\t  -20.25 \t Z \t  :\t  30.0\n",
            b"ok\n"
        ]
        pos = connection.get_position()
        assert pos == (10.5, -20.25, 30.0)

        # Scenario B: No spacing separator between label and value (e.g. X10.5)
        # Current pattern: r"X\s*[:\s]\s*..." -> requires a `:` or a whitespace character as a separator.
        # This test checks if X10.5 Y20.5 Z30.5 fails or succeeds.
        mock_serial_instance.readline.side_effect = [
            b"X10.5 Y20.5 Z30.5\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError) as exc_info:
            connection.get_position()
        assert "Coordinates not found in response" in str(exc_info.value)

        # Scenario C: Trailing units or other coordinates (e.g. mm or extra axis E)
        # Trailing characters like mm will cause regex failure because pattern expects Y directly after X value.
        mock_serial_instance.readline.side_effect = [
            b"X: 10.0mm Y: 20.0mm Z: 30.0mm\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

        # Extra axis at the end matches if Z matches.
        mock_serial_instance.readline.side_effect = [
            b"X: 10.0 Y: 20.0 Z: 30.0 E: 0.0\n",
            b"ok\n"
        ]
        pos = connection.get_position()
        assert pos == (10.0, 20.0, 30.0)

        # Extra axis inside Z will fail.
        mock_serial_instance.readline.side_effect = [
            b"X: 10.0 Y: 20.0 E: 0.0 Z: 30.0\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

        # Scenario D: Missing coordinate fields (e.g., Y and Z only)
        mock_serial_instance.readline.side_effect = [
            b"Y: 20.0 Z: 30.0\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

        # Scenario E: Scientific notation
        mock_serial_instance.readline.side_effect = [
            b"X: 1.0e1 Y: 2.0e1 Z: 3.0e1\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

        # Scenario F: Non-numeric values
        mock_serial_instance.readline.side_effect = [
            b"X: ten Y: twenty Z: thirty\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

        # Scenario G: Coordinates split across multiple lines
        mock_serial_instance.readline.side_effect = [
            b"X: 10.0\n",
            b"Y: 20.0\n",
            b"Z: 30.0\n",
            b"ok\n"
        ]
        with pytest.raises(RuntimeError):
            connection.get_position()

# --- 3. Concurrency and Lock Behavior ---

class SimulatedSlowSerial:
    """
    Simulates a thread-safe-ish serial port that asserts it is not accessed concurrently by multiple writes
    and simulates realistic timing to catch race conditions.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.active_write = False
        self.responses = []
        self.written_commands = []
        self.is_open = True

    def write(self, data):
        # Verify that only one thread can execute write at a time
        acquired = self.lock.acquire(blocking=False)
        if not acquired:
            raise AssertionError("Concurrent write detected on Serial port!")
        try:
            self.written_commands.append(data)
            time.sleep(0.01) # Simulate transmission time
        finally:
            self.lock.release()

    def readline(self):
        if self.responses:
            return self.responses.pop(0)
        return b""

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def close(self):
        self.is_open = False

@patch("serial.Serial")
def test_send_thread_concurrency(mock_serial_cls, connection):
    """
    Tests GCodeConnection's lock behavior when multiple threads send commands concurrently.
    """
    sim_serial = SimulatedSlowSerial()
    mock_serial_cls.return_value = sim_serial
    
    # We populate the simulated responses.
    # Each command sent expects "ok\n"
    num_threads = 10
    sim_serial.responses = [b"ok\n"] * num_threads

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()

        errors = []
        results = []

        def worker(thread_idx):
            try:
                # Send command unique to this thread
                res = connection.send(f"G1 F{thread_idx}")
                results.append((thread_idx, res))
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(num_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Check for concurrency errors
        assert len(errors) == 0, f"Exceptions encountered: {errors}"
        assert len(sim_serial.written_commands) == num_threads
        assert len(results) == num_threads
        for idx, res in results:
            assert res == ["ok"]

# --- 4. High-Level Non-Atomicity Vulnerability ---

@patch("serial.Serial")
def test_high_level_non_atomicity(mock_serial_cls, connection):
    """
    Demonstrates that high-level operations (like move_and_verify or vacuum_off)
    are NOT atomic, even though low-level `send` is thread-safe.
    """
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance

    # Thread A: calls move_and_verify(10, 10, 10, 1000)
    # Thread B: calls move_and_verify(99, 99, 99, 9999)
    #
    # We simulate an interleaved execution sequence:
    # 1. Thread A calls self.move -> writes G1 X10 Y10 Z10 F1000 and M400, receives ok, ok.
    # 2. Thread B calls self.move -> writes G1 X99 Y99 Z99 F9999 and M400, receives ok, ok.
    # 3. Thread A calls get_position -> writes M1008 A3, receives Thread B's position: X:99 Y:99 Z:99.
    # 4. Thread B calls get_position -> writes M1008 A3, receives Thread B's position: X:99 Y:99 Z:99.
    #
    # Expected result: Thread A fails move_and_verify because it reads Thread B's coordinates!
    
    mock_serial_instance.readline.side_effect = [
        # Thread A move:
        b"ok\n", b"ok\n",
        # Thread B move:
        b"ok\n", b"ok\n",
        # Thread A get_position:
        b"X:99.0 Y:99.0 Z:99.0\n", b"ok\n",
        # Thread B get_position:
        b"X:99.0 Y:99.0 Z:99.0\n", b"ok\n",
    ]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()

        thread_a_result = []
        thread_b_result = []

        # We will use simple events to force this exact interleaving sequence
        event_a_moved = threading.Event()
        event_b_moved = threading.Event()

        thread_a_ident = None

        def worker_a():
            nonlocal thread_a_ident
            thread_a_ident = threading.get_ident()
            res = connection.move_and_verify(10.0, 10.0, 10.0, 1000)
            thread_a_result.append(res)

        def worker_b():
            event_a_moved.wait() # Wait until Thread A has sent its move command
            res = connection.move_and_verify(99.0, 99.0, 99.0, 9999)
            thread_b_result.append(res)
            event_b_moved.set() # Allow Thread A to proceed to get_position

        # Wrap get_position to check thread identities and coordinate progress
        original_get_position = connection.get_position
        def wrapped_get_position():
            if threading.get_ident() == thread_a_ident:
                event_a_moved.set()
                event_b_moved.wait() # Wait until Thread B has also moved and verified
            return original_get_position()

        connection.get_position = wrapped_get_position

        try:
            t_a = threading.Thread(target=worker_a)
            t_b = threading.Thread(target=worker_b)

            t_a.start()
            t_b.start()

            t_a.join()
            t_b.join()
        finally:
            connection.get_position = original_get_position

        # Thread A should fail verification because Thread B overwrote the position before Thread A could verify it.
        assert thread_a_result[0] is False
        assert thread_b_result[0] is True
