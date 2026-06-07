# huenit_vision

**Vision‑gestützte Steuer‑ und Kalibrierungs‑Bibliothek für den Huenit‑Roboter‑Arm**

## Überblick
`huenit_vision` ist ein eigenständiges Python‑Paket, das alle Komponenten für:
- Kamerabasierte Marker‑Erkennung (ArUco)
- Kalibrierung einer Hand‑/Deckenkamera zu den Roboter‑Koordinaten
- Koordinatentransformation zwischen Bild‑Pixel‑ und Roboter‑Raum
- Pick‑ und Place‑Sequenzen mit visuellem Servoing und Verifikation

Das Paket ist **modular** aufgebaut und greift ausschließlich auf die **inneren Module** (`config`, `serial_comm`, `cameras`, …) des Pakets zu. Es hat keine Laufzeit‑Abhängigkeit zu den übergeordneten Verzeichnissen (z. B. `robot_control`). Damit kann es in jedem Python‑Projekt importiert werden, das das Verzeichnis `huenit_vision` im `PYTHONPATH` hat.

## Projektstruktur
```
huenit_vision/
├─ __init__.py                # Paket‑Initialisierung
├─ config.py                  # immutable RobotConfig Dataclass
├─ serial_comm.py             # G‑Code‑Schnittstelle (move_and_verify, vacuum …)
├─ cameras.py                 # CameraManager (overhead + hand)
├─ aruco.py                   # Marker‑Detection (ID 0)
├─ workspace.py               # Kinematik‑Hilfsfunktionen
├─ calibration/
│   ├─ __init__.py           # exports grid_mapper, hand_camera_servoing, coordinate_transform
│   ├─ grid_mapper.py        # BFS‑Kalibrierung, Daten‑Persistenz
│   ├─ hand_camera_servoing.py# visuelles Servo‑Modul
│   └─ coordinate_transform.py # affine Pixel↔Robot‑Mapping
└─ operations/
    ├─ __init__.py           # exports pick, place
    ├─ pick.py               # robustes Pick‑Verfahren mit Verifikation
    └─ place.py              # Place‑Sequenz mit Blow‑Off‑Option
```

## Installation
```bash
# im Projekt‑Root (robot-arm-video-feedback) ausführen
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt   # OpenCV, numpy etc.
# Optional: als editierbares Paket installieren
pip install -e ./huenit_vision
```
> **Hinweis:** Das Paket benötigt `opencv-python>=4.6`, `numpy>=1.24` und einen laufenden Serial‑Port für die Firmware. Die Hardware‑Tests werden im `development`‑Modus durchgeführt.

## Grundlegende Nutzung
```python
from huenit_vision.config import RobotConfig
from huenit_vision.serial_comm import GCodeConnection
from huenit_vision.cameras import CameraManager
from huenit_vision.calibration.grid_mapper import GridMapper
from huenit_vision.calibration.hand_camera_servoing import HandCameraServoing
from huenit_vision.operations.pick import pick_object
from huenit_vision.operations.place import place_object

# Initialisierung
cfg = RobotConfig()
conn = GCodeConnection(cfg.serial_port, cfg.baudrate)
cam_mgr = CameraManager()

# Kalibrierung (einmalig)
mapper = GridMapper(conn, cam_mgr, cfg)
mapper.run()  # legt grid.json im Projektordner an

# Visual Servoing (nach Kalibrierung)
servo = HandCameraServoing(cam_mgr, mapper.transform)

# Pick‑Beispiel
success = pick_object(conn, cam_mgr, servo, x=120.0, y=80.0,
                       Z_pick=cfg.z_pick, Z_travel=cfg.z_travel, config=cfg)

# Place‑Beispiel
if success:
    place_object(conn, x=200.0, y=150.0, Z_place=cfg.z_place,
                 Z_travel=cfg.z_travel, config=cfg)
```

## Unabhängigkeit vom übergeordneten Verzeichnis
- Das Paket verwendet ausschließlich **relative Importe** innerhalb `huenit_vision`.
- Keine Datei aus `../robot_control` oder anderen Projektrepositories wird importiert.
- Der einzige externe Bezug besteht in der *optional* installierbaren `requirements.txt`.
- Damit kann das Paket problemlos in anderen Projekten wiederverwendet werden, solange die Hardware‑Schnittstelle (`GCodeConnection`) bereitgestellt wird.

## Testing
```bash
pytest -q huenit_vision/tests
```
Alle Unit‑Tests befinden sich im Unterordner `tests/` und prüfen Marker‑Detektion, Kalibrierungs‑Workflow sowie Pick/Place‑Logik.

## Lizenz
Dieses Paket ist unter der MIT‑Lizenz veröffentlicht – siehe `LICENSE` im Projekt‑Root.

---
*Erstellt von Antigravity, automatischer Code‑Assistent.*
