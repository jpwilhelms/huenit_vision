import time
import pytest
import warnings
from unittest.mock import MagicMock, patch
import serial
from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection

@pytest.fixture
def mock_config():
    # Keep timeouts small in test for quick execution
    return RobotConfig(
        baudrate=115200,
        serial_timeout=0.1,
        gcode_response_timeout=0.2,
        position_tolerance_mm=1.0,
        vacuum_release_time=0.1
    )

@pytest.fixture
def connection(mock_config):
    return GCodeConnection(mock_config)

class MockPortInfo:
    def __init__(self, device, description, vid=None, pid=None, manufacturer=None, serial_number=None):
        self.device = device
        self.description = description
        self.vid = vid
        self.pid = pid
        self.manufacturer = manufacturer
        self.serial_number = serial_number

# --- Connection Management Tests ---

def test_scan_ports(connection):
    mock_ports = [
        MockPortInfo("/dev/ttyUSB0", "FTDI USB Serial", manufacturer="FTDI"),
        MockPortInfo("/dev/ttyUSB1", "Generic COM", manufacturer="Generic"),
    ]
    with patch("serial.tools.list_ports.comports", return_value=mock_ports):
        ports = connection.scan_ports()
        assert len(ports) == 2
        assert ports[0][0] == "/dev/ttyUSB0"
        assert ports[0][4] == "FTDI"
        assert ports[1][0] == "/dev/ttyUSB1"

def test_discover_port_ftdi_manufacturer(connection):
    mock_ports = [
        MockPortInfo("/dev/ttyUSB1", "Generic COM", manufacturer="Generic"),
        MockPortInfo("/dev/ttyUSB0", "FTDI USB Serial", manufacturer="FTDI"),
    ]
    with patch("serial.tools.list_ports.comports", return_value=mock_ports):
        port = connection.discover_port()
        assert port == "/dev/ttyUSB0"

def test_discover_port_ftdi_description(connection):
    mock_ports = [
        MockPortInfo("/dev/ttyUSB1", "Generic COM", manufacturer="Generic"),
        MockPortInfo("/dev/ttyUSB0", "USB Serial Device (FTDI)", manufacturer=None),
    ]
    with patch("serial.tools.list_ports.comports", return_value=mock_ports):
        port = connection.discover_port()
        assert port == "/dev/ttyUSB0"

def test_discover_port_fallback(connection):
    mock_ports = [
        MockPortInfo("/dev/ttyUSB1", "Generic COM", manufacturer="Generic"),
        MockPortInfo("/dev/ttyUSB2", "Another COM", manufacturer="Other"),
    ]
    with patch("serial.tools.list_ports.comports", return_value=mock_ports):
        port = connection.discover_port()
        assert port == "/dev/ttyUSB1"

def test_discover_port_no_ports(connection):
    with patch("serial.tools.list_ports.comports", return_value=[]):
        port = connection.discover_port()
        assert port is None

@patch("serial.Serial")
def test_open_success(mock_serial_cls, connection, mock_config):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        assert connection.is_connected()
        assert connection.port == "/dev/ttyUSB0"
        
        # Verify constructor called
        mock_serial_cls.assert_called_once_with(
            port="/dev/ttyUSB0",
            baudrate=mock_config.baudrate,
            timeout=mock_config.serial_timeout
        )
        # Verify RTS set to False
        assert mock_serial_instance.rts is False

@patch("serial.Serial")
def test_open_already_open(mock_serial_cls, connection):
    mock_serial_instance_1 = MagicMock()
    mock_serial_instance_2 = MagicMock()
    mock_serial_cls.side_effect = [mock_serial_instance_1, mock_serial_instance_2]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        assert connection.is_connected()
        # Open again
        connection.open()
        mock_serial_instance_1.close.assert_called_once()
        assert connection.is_connected()

@patch("serial.Serial")
def test_open_failure(mock_serial_cls, connection):
    mock_serial_cls.side_effect = serial.SerialException("Access denied")
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        with pytest.raises(RuntimeError) as exc_info:
            connection.open()
        assert "Failed to open connection to /dev/ttyUSB0" in str(exc_info.value)
        assert not connection.is_connected()
        assert connection.port is None

@patch("serial.Serial")
def test_close(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        assert connection.is_connected()
        
        connection.close()
        assert not connection.is_connected()
        assert connection.port is None
        mock_serial_instance.close.assert_called_once()

# --- Send and Parsing Tests ---

@patch("serial.Serial")
def test_send_single_command(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.side_effect = [b"ok\n"]
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        responses = connection.send("G0 X100")
        mock_serial_instance.write.assert_called_once_with(b"G0 X100\n")
        assert responses == ["ok"]

@patch("serial.Serial")
def test_send_multiple_commands(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.side_effect = [
        b"echo: G1 X10\n", b"ok\n",
        b"ok\n"
    ]
    
    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        responses = connection.send("G1 X10\nM400")
        assert mock_serial_instance.write.call_count == 2
        mock_serial_instance.write.assert_any_call(b"G1 X10\n")
        mock_serial_instance.write.assert_any_call(b"M400\n")
        assert responses == ["echo: G1 X10", "ok", "ok"]

@patch("serial.Serial")
def test_send_not_connected(mock_serial_cls, connection):
    with pytest.raises(RuntimeError) as exc_info:
        connection.send("G0 X100")
    assert "Serial connection is not open." in str(exc_info.value)

@patch("serial.Serial")
def test_send_timeout_warning(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    # Simulate serial readline timeout returning empty bytes indefinitely
    mock_serial_instance.readline.return_value = b""

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        with pytest.warns(RuntimeWarning) as record:
            connection.send("G0 X100")
        
        assert len(record) > 0
        assert "Timeout" in str(record[0].message)

# --- get_position and Parsing Variant Tests ---

@patch("serial.Serial")
def test_get_position_success_variants(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    variants = [
        ([b"Current Point: X:10.50 Y:-20.25 Z:30.00\n", b"ok\n"], (10.5, -20.25, 30.0)),
        ([b"X: 0 Y: 180 Z: 0\n", b"ok\n"], (0.0, 180.0, 0.0)),
        ([b"echo: M1008 A3\n", b"X:+123.456 Y: -789.0 Z:0.0\n", b"ok\n"], (123.456, -789.0, 0.0)),
        ([b"x: 10.0 y: 20.0 z: 30.0\n", b"ok\n"], (10.0, 20.0, 30.0))
    ]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        
        for lines, expected in variants:
            mock_serial_instance.readline.side_effect = lines
            pos = connection.get_position()
            assert pos == expected

@patch("serial.Serial")
def test_get_position_failure(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.side_effect = [b"Current Point: none\n", b"ok\n"]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        with pytest.raises(RuntimeError) as exc_info:
            connection.get_position()
        assert "Coordinates not found in response" in str(exc_info.value)

# --- Hardware Control Tests ---

@patch("serial.Serial")
@patch("time.sleep", return_value=None)
def test_home(mock_sleep, mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.home()
        
        # Verify commands sent in order
        calls = mock_serial_instance.write.call_args_list
        assert calls[0][0][0] == b"M1008 A5\n"
        assert calls[1][0][0] == b"M17\n"
        assert calls[2][0][0] == b"G90\n"
        
        # Verify sleep called
        mock_sleep.assert_any_call(1.0)

@patch("serial.Serial")
def test_enable_motors(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.enable_motors()
        mock_serial_instance.write.assert_called_with(b"M17\n")

@patch("serial.Serial")
def test_vacuum_on(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.vacuum_on()
        
        calls = mock_serial_instance.write.call_args_list
        assert calls[0][0][0] == b"M1401 A0\n"
        assert calls[1][0][0] == f"M1400 A{connection.config.pump_max_power}\n".encode('utf-8')

@patch("serial.Serial")
@patch("time.sleep", return_value=None)
def test_vacuum_off_with_blow_off(mock_sleep, mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.vacuum_off(blow_off=True)
        
        calls = mock_serial_instance.write.call_args_list
        assert calls[0][0][0] == b"M1400 A0\n"
        assert calls[1][0][0] == b"M1401 A1\n"
        assert calls[2][0][0] == b"M1401 A0\n"
        
        mock_sleep.assert_called_with(connection.config.vacuum_release_time)

@patch("serial.Serial")
@patch("time.sleep", return_value=None)
def test_vacuum_off_no_blow_off(mock_sleep, mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.vacuum_off(blow_off=False)
        
        mock_serial_instance.write.assert_called_once_with(b"M1400 A0\n")
        mock_sleep.assert_not_called()

# --- Move and Verify Tests ---

@patch("serial.Serial")
def test_move(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    mock_serial_instance.readline.return_value = b"ok\n"

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        connection.move(10.0, 20.0, 30.0, 8000)
        
        calls = mock_serial_instance.write.call_args_list
        assert calls[0][0][0] == b"G1 X10.0 Y20.0 Z30.0 F8000\n"
        assert calls[1][0][0] == b"M400\n"

@patch("serial.Serial")
def test_move_and_verify_success(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    # 1. move writes G1 and M400, receives ok, ok
    # 2. get_position writes M1008 A3, receives X:10.5 Y:19.8 Z:30.2, ok
    mock_serial_instance.readline.side_effect = [
        b"ok\n", b"ok\n", # response to G1, M400
        b"X:10.5 Y:19.8 Z:30.2\n", b"ok\n" # response to M1008 A3
    ]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        res = connection.move_and_verify(10.0, 20.0, 30.0, 8000)
        assert res is True  # differences are 0.5, 0.2, 0.2 which are <= 1.0

@patch("serial.Serial")
def test_move_and_verify_out_of_tolerance(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    # 1. move writes G1 and M400, receives ok, ok
    # 2. get_position writes M1008 A3, receives X:12.0 Y:20.0 Z:30.0, ok
    mock_serial_instance.readline.side_effect = [
        b"ok\n", b"ok\n",
        b"X:12.0 Y:20.0 Z:30.0\n", b"ok\n"
    ]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        res = connection.move_and_verify(10.0, 20.0, 30.0, 8000)
        assert res is False  # X difference is 2.0 > 1.0

@patch("serial.Serial")
def test_move_and_verify_get_position_exception(mock_serial_cls, connection):
    mock_serial_instance = MagicMock()
    mock_serial_cls.return_value = mock_serial_instance
    
    # get_position fails to return coordinates (raises Exception)
    mock_serial_instance.readline.side_effect = [
        b"ok\n", b"ok\n",
        b"ok\n" # no coordinates returned
    ]

    with patch.object(connection, "discover_port", return_value="/dev/ttyUSB0"):
        connection.open()
        res = connection.move_and_verify(10.0, 20.0, 30.0, 8000)
        assert res is False
