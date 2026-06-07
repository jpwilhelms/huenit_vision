# E2E Test Infra: huenit_vision

## Test Philosophy
- Opaque-box, requirement-driven. Supports both simulated mock execution and physical hardware execution.
- Methodology: Category-Partition + BVA + Pairwise + Workload Testing.

## Feature Inventory
| # | Feature | Source (requirement) | Tier 1 | Tier 2 | Tier 3 |
|---|---------|---------------------|:------:|:------:|:------:|
| 1 | Central Configuration (`RobotConfig`) | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 2 | Serial G-Code Connection (`GCodeConnection`) | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 3 | Camera Management (`CameraManager`) | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ |
| 4 | Marker ID 0 Detection (`detect_marker_0`) | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ |
| 5 | Workspace Bounds Check (`check_reachable`) | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ |
| 6 | Affine Transformation (`CoordinateTransform`) | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |
| 7 | Visual Servoing (`HandCameraServoing`) | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |
| 8 | BFS Gitter-Kalibrierung (`GridMapper`) | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ |
| 9 | Pick Operation (`pick_object`) | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ |
| 10| Place Operation (`place_object`) | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ |

## Test Architecture
- Test runner: `pytest`
- Test files located in `huenit_vision/tests/` (e.g., `test_config.py`, `test_workspace.py`, `test_aruco.py`, `test_serial_comm.py`, etc.)
- Mock objects for `serial.Serial` and `cv2.VideoCapture` are defined to simulate serial communication and camera capture.

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Complexity |
|---|----------|--------------------|------------|
| 1 | Full BFS Grid Calibration | RobotConfig, GCodeConnection, CameraManager, CoordinateTransform, HandCameraServoing, GridMapper | High |
| 2 | Pick and Place with Verification | RobotConfig, GCodeConnection, CameraManager, HandCameraServoing, pick_object, place_object | High |

## Coverage Thresholds
- Tier 1: ≥5 per feature
- Tier 2: ≥5 per feature (where boundaries exist)
- Tier 3: pairwise coverage of major feature interactions
- Tier 4: ≥5 realistic application scenarios
