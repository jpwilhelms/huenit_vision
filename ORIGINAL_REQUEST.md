# Original User Request

## Initial Request — 2026-06-07T09:09:20+02:00

Implementierung einer modularen Python-Bibliothek namens `huenit_vision` im Verzeichnis `/home/jpw/ai-projects/robot-arm-video-feedback/huenit_vision/` gemäß der Spezifikation in `/home/jpw/.gemini/antigravity-cli/brain/b594808f-3947-4101-8269-f16a497c48b4/huenit_vision_spec.md`.

Die Library muss komplett eigenständig implementiert werden (keine Wiederverwendung oder Importe aus dem bestehenden Ordner `robot_control`).
Die Verifikation erfolgt direkt über Hardware-Tests im venv des Projekts.

## Requirements

### R1. config.py — Zentrale Konfiguration
Definition der Klasse `RobotConfig` als `@dataclass` mit allen 25 empirischen Parametern aus der Spezifikation (Abschnitt 3.1). Keine Magic Numbers in anderen Modulen.

### R2. serial_comm.py — Serielle Kommunikation
Implementierung der Klasse `GCodeConnection` mit synchroner G-Code-Übertragung.
- Jede G-Code-Zeile einzeln senden, auf das "ok" der Firmware warten (inkl. Timeout-Behandlung).
- Für Bewegungsbefehle zusätzlich `M400` senden und auf dessen "ok" warten.
- `move_and_verify`: Soll-Ist-Abgleich der Roboterposition nach Bewegungen via `get_position()` (welches `M1008 A3` sendet und auswertet). Abbruch bei Toleranzüberschreitung.
- `vacuum_off(blow_off=True)`: Vakuumsteuerung mit optionalem Belüftungsventil (`M1401`). Falls `blow_off=False` (z.B. nach fehlgeschlagenem Pick), das Belüftungsventil geschlossen lassen, um Werkstücke nicht zu verschieben.
- `home()`: Homing-Sequenz mit `M1008 A5`, Motoren aktivieren (`M17`) und absolutem Modus (`G90`).

### R3. cameras.py & aruco.py — Bildverarbeitung und Marker-Erkennung
- `CameraManager`: Einmaliges Öffnen beider USB-Kameras (`overhead` auf ID 2, `hand` auf ID 6) mit `BUFFERSIZE=1`, `MJPG` und Auflösungen aus Config. Schließen erst bei Programmende.
- Frame-Flushing: Vor Bildaufnahmen 8 Frames verwerfen, um Ringpuffer-Latenzen zu eliminieren.
- `detect_marker_0(frame)`: Erkennung von ArUco-Marker-ID 0 über 4 Dictionaries (`DICT_4X4_50`, `DICT_4X4_100`, `DICT_6X6_50`, `DICT_APRILTAG_36h11`) unter Beachtung der OpenCV-Versionsunterschiede (4.6 vs 4.7+ API).

### R4. workspace.py — Kinematische Prüfung
- `check_reachable(x, y, z)`: Mathematische Approximation des Arbeitsraums des Roboterarms (reimplementiert ohne Importe aus `robot_control`).

### R5. calibration/ — Kalibrierungs-Algorithmen
- `coordinate_transform.py`: Transformation zwischen Pixel- und Roboterkoordinaten mittels affiner Transformation (`cv2.estimateAffine2D`). Speichern/Laden der Transformationsmatrizen.
- `hand_camera_servoing.py`: Iteratives Visual Servoing (Proportionalregler mit 0.12 mm/Pixel Gain, Schrittbegrenzung auf 5.0mm, Toleranz 4px) bei Z=77.3.
- `grid_mapper.py`: Automatische BFS-basierte Gitterkalibrierung (Schrittweite 30mm).
  - Inkrementelles Speichern der JSON-Ergebnisse nach jedem Messpunkt.
  - Vollständiger Schutz durch `try...finally`, um bei Abbruch das Vakuum abzuschalten und Verbindungen/Kameras sauber freizugeben.

### R6. operations/ — Pick-and-Place-Abläufe
- `pick.py`: Visual Servoing zur Feinjustierung, Absenken auf `Z_pick` (Kompression -4.5mm), Vakuum aktivieren, Anheben und Verifikation mittels *Deckenkamera aus der Parkposition* (um das Herunterfallen von Objekten aus 10cm Höhe zu verhindern). Bei Fehlpick `vacuum_off(blow_off=False)`.
- `place.py`: Fahrt auf `Z_travel`, Absenken auf `Z_place` (Fallhöhe +1.0mm, um Blockieren der Belüftung zu verhindern), Vakuum aus mit `blow_off=True`.

## Acceptance Criteria

### Funktionalität und Schnittstellen
- [ ] Alle spezifizierten Klassen, Methoden und Funktionen sind vorhanden, typisiert und importierbar.
- [ ] Der Code enthält keine Magic Numbers außerhalb von `config.py`.
- [ ] Alle Hardware-Timing-Vorgaben (0.5s Vakuumaufbau, 1.5s Vakuumabbau, 0.5s Kamera-Beruhigungszeit, 8 Frames Flushing) sind exakt implementiert.
- [ ] Ein Testskript, das alle Module importiert und einfache Operationen durchführt, läuft ohne Syntax- oder Importfehler durch.
- [ ] Der Cleanup-Mechanismus in `grid_mapper.py` und allen Hauptfunktionen schaltet bei Fehlern oder Abbruch das Vakuum sicher ab und gibt die Kameras frei.
