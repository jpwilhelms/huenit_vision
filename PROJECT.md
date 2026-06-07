# Project: huenit_vision

## Architecture
- `huenit_vision/config.py`: RobotConfig dataclass
- `huenit_vision/workspace.py`: check_reachable workspace check
- `huenit_vision/aruco.py`: detect_marker_0 detecting ArUco marker ID 0
- `huenit_vision/serial_comm.py`: GCodeConnection class
- `huenit_vision/cameras.py`: CameraManager class
- `huenit_vision/calibration/coordinate_transform.py`: CoordinateTransform class
- `huenit_vision/calibration/hand_camera_servoing.py`: HandCameraServoing class
- `huenit_vision/calibration/grid_mapper.py`: GridMapper class
- `huenit_vision/operations/pick.py`: pick_object function
- `huenit_vision/operations/place.py`: place_object function

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|---|---|---|---|
| 1 | config, workspace & aruco | config.py, workspace.py, aruco.py | none | DONE |
| 2 | serial_comm | serial_comm.py | M1 | IN_PROGRESS |
| 3 | cameras | cameras.py | M1 | PLANNED |
| 4 | calibration | calibration/ (coordinate_transform, hand_camera_servoing, grid_mapper) | M1, M2, M3 | PLANNED |
| 5 | operations | operations/ (pick, place) | M1, M2, M3, M4 | PLANNED |
| 6 | packaging & validation | __init__.py and full test verification | M5 | PLANNED |

## Interface Contracts
### GCodeConnection ↔ RobotConfig
- GCodeConnection takes a RobotConfig instance at initialization.
- Uses configs for baudrate, timeouts, feed rates, limits, and timing.

### CameraManager ↔ RobotConfig
- CameraManager uses RobotConfig camera IDs, resolutions, buffers, and flush counts.

### HandCameraServoing ↔ GCodeConnection & CameraManager & RobotConfig
- HandCameraServoing uses GCodeConnection for movements, CameraManager for capturing frames, and RobotConfig for parameters.

### GridMapper ↔ GCodeConnection & CameraManager & HandCameraServoing & RobotConfig
- BFS-based mapper orchestrating movements, picks, and camera captures for grid calibration.

### Operations (pick, place) ↔ GCodeConnection & CameraManager & HandCameraServoing & RobotConfig
- Fine adjust, pick verification, placing, vacuum off with blow_off option.

## Code Layout
- Target folder: `/home/jpw/ai-projects/robot-arm-video-feedback/huenit_vision/`
