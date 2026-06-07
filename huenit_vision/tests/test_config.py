import os
import pytest
from dataclasses import FrozenInstanceError
from huenit_vision.config import RobotConfig

def test_default_config():
    config = RobotConfig()
    assert config.baudrate == 115200
    assert config.z_verify == 77.3
    assert config.park_x == -150.0
    # verify some other default parameters
    assert config.overhead_camera_id == 2
    assert config.hand_camera_id == 6
    assert config.overhead_width == 1280
    assert config.overhead_height == 720
    assert config.hand_width == 640
    assert config.hand_height == 480
    assert config.camera_buffer_size == 1
    assert config.frame_flush_count == 8
    assert config.camera_settle_time == 0.5
    assert config.camera_warmup_frames == 15
    assert config.z_place_offset == 1.0
    assert config.z_pick_offset == -4.5
    assert config.z_travel_offset == 40.0
    assert config.park_y == 150.0
    assert config.vacuum_buildup_time == 0.5
    assert config.vacuum_release_time == 1.5
    assert config.pump_max_power == 1023
    assert config.pump_hold_power == 623
    assert config.feed_travel == 8000
    assert config.feed_vertical == 4000
    assert config.feed_servoing == 5000
    assert config.servoing_gain_x == -0.12
    assert config.servoing_gain_y == 0.12
    assert config.servoing_tolerance_px == 4.0
    assert config.servoing_max_steps == 5
    assert config.servoing_step_limit_mm == 5.0
    assert config.position_tolerance_mm == 1.0
    assert config.max_pick_retries == 3
    assert config.grid_step_mm == 30.0

def test_immutability():
    config = RobotConfig()
    with pytest.raises(FrozenInstanceError):
        config.baudrate = 9600  # type: ignore

def test_height_helpers():
    config = RobotConfig()
    z_start = 10.0
    assert config.get_z_place(z_start) == 11.0
    assert config.get_z_pick(z_start) == 5.5
    assert config.get_z_travel(z_start) == 50.0

def test_gain_scaling():
    config = RobotConfig()
    # At exact z_verify height, gains should match defaults
    gx, gy = config.get_servoing_gains(77.3)
    assert pytest.approx(gx) == -0.12
    assert pytest.approx(gy) == 0.12
    
    # At double z_verify height, gains should double
    gx_double, gy_double = config.get_servoing_gains(154.6)
    assert pytest.approx(gx_double) == -0.24
    assert pytest.approx(gy_double) == 0.24
    
    # At z_verify = 0, should return default gains without DivisionByZero
    zero_verify_config = RobotConfig(z_verify=0.0)
    gx_zero, gy_zero = zero_verify_config.get_servoing_gains(100.0)
    assert gx_zero == -0.12
    assert gy_zero == 0.12

def test_json_serialization(tmp_path):
    config = RobotConfig(baudrate=9600, z_verify=80.0)
    filepath = os.path.join(tmp_path, "config.json")
    
    config.save_to_json(filepath)
    loaded_config = RobotConfig.load_from_json(filepath)
    
    assert loaded_config.baudrate == 9600
    assert loaded_config.z_verify == 80.0
    assert loaded_config == config
    
    # Check serialization/deserialization dictionary roundtrip
    d = config.to_dict()
    assert d["baudrate"] == 9600
    assert d["z_verify"] == 80.0
    
    loaded_from_dict = RobotConfig.from_dict(d)
    assert loaded_from_dict == config

def test_load_from_json_not_found():
    with pytest.raises(FileNotFoundError):
        RobotConfig.load_from_json("nonexistent_file.json")

def test_config_type_coercion():
    # Coercing a mix of float, int, str for int and float fields, and int | str fields
    raw_data = {
        "baudrate": "115200",               # expected int
        "serial_timeout": 2,                # expected float
        "overhead_camera_id": 2.0,          # expected int | str -> int
        "hand_camera_id": "/dev/video6",    # expected int | str -> str
        "camera_settle_time": "0.75",       # expected float
        "z_place_offset": "1",              # expected float
    }
    config = RobotConfig.from_dict(raw_data)
    assert config.baudrate == 115200
    assert isinstance(config.baudrate, int)
    
    assert config.serial_timeout == 2.0
    assert isinstance(config.serial_timeout, float)
    
    assert config.overhead_camera_id == 2
    assert isinstance(config.overhead_camera_id, int)
    
    assert config.hand_camera_id == "/dev/video6"
    assert isinstance(config.hand_camera_id, str)
    
    assert config.camera_settle_time == 0.75
    assert isinstance(config.camera_settle_time, float)
    
    assert config.z_place_offset == 1.0
    assert isinstance(config.z_place_offset, float)

