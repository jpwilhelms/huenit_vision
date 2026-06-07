# Roboterarm Video-Feedback & Kalibrierung

Dieses Verzeichnis enthält die Python-Infrastruktur zur Steuerung des Huenit-Roboterarms sowie zur Kalibrierung der angeschlossenen Kamerasysteme.

## Systemarchitektur & Kamera-Konfiguration

Das System arbeitet mit einer festen Konfiguration aus drei USB-Kameras:
- **`/dev/video0`**: Seitenkamera (Side Camera), Auflösung: `640x480`
- **`/dev/video2`**: Deckenkamera (Top/Overhead Camera), MJPEG-Stream, Auflösung: `1280x720` (bzw. `640x480` im Kompatibilitätsmodus)
- **`/dev/video6`**: Handkamera (Hand Camera), montiert am Effektor des Arms, Auflösung: `640x480`

---

## Hardware-Steuerung & Protokoll (`robot_control`)

Die Kommunikation mit dem Roboterarm erfolgt seriell über eine FTDI-Schnittstelle bei einer Baudrate von `115200`. Das Low-Level-Protokoll basiert auf einem standardisierten G-Code-Dialekt.

Die Schnittstelle ist in [robot.py](file:///home/jpw/ai-projects/robot-arm-video-feedback/robot_control/robot.py) implementiert. Folgende Befehle und Funktionen sind für die Kalibrierung kritisch:

### Serielle Befehle (G-Code & M-Codes)
- **`M17`**: Aktiviert die Schrittmotoren.
- **`M84`**: Deaktiviert die Motoren (Freilauf/Free Mode).
- **`G90`**: Setzt den Positionierungsmodus auf absolute Koordinaten.
- **`M1008 A5`**: Führt ein Homing (Referenzfahrt) aller Achsen durch.
- **`G0` / `G1 X[x] Y[y] Z[z] F[f]`**: Linearbewegungen in Millimetern. `F` definiert die Feedrate (Geschwindigkeit in mm/min).
- **`M400`**: Wartet mit der Verarbeitung nachfolgender Befehle, bis alle Bewegungen im Puffer abgeschlossen sind (Motion-Queue-Sync).
- **`M1400 A1023` & `M1401 A0`**: Aktiviert die Vakuumpumpe (Suction ON).
- **`M1400 A0` & `M1401 A1`**: Schaltet die Vakuumpumpe ab und aktiviert kurzzeitig das Ausblasventil zum Lösen des Objekts (Suction OFF).

### Kinematische Workspace-Validierung (`checkXYZ`)
Vor jedem Bewegungsbefehl validiert die Funktion `checkXYZ(x, y, z)` geometrisch, ob die Zielkoordinaten im erreichbaren Arbeitsraum des SCARA-basierten Arms liegen. Dies verhindert mechanische Kollisionen und Schrittverluste an den Gelenkanschlägen. Der maximale Arbeitsbereich in Y liegt bei ca. `130.0` bis `290.0` mm.

---

## Kalibrierungsmethoden (Skripte)

Es stehen drei verschiedene Kalibrierungsskripte zur Verfügung, die auf unterschiedlichen sensorischen Feedbackschleifen aufbauen.

### 1. Physische Scheibenkalibrierung ([calibrate_overhead_discs.py](file:///home/jpw/ai-projects/robot-arm-video-feedback/calibrate_overhead_discs.py))
Diese Methode berechnet die affine Transformation der Deckenkamera mittels eines weißen runden Kalibrierobjekts (Scheibe) mit schwarzem Zentrum.

- **Konzept**: Der Roboter platziert das Objekt präzise an vordefinierten Punkten, zieht sich zurück, erfasst das Objekt mit der Deckenkamera und holt es wieder ab.
- **Ablauf**:
  1. **Suchlauf (Step 0)**: Der Arm scannt ein Suchraster über die Handkamera ab, bis er die Scheibe detektiert.
  2. **Fein-Zentrierung (Visual Servoing)**: Der Arm zentriert sich über dem Schwarzpunkt der Scheibe (Proportional-Regelung über Fehlerpixel) und greift diese.
  3. **Kalibrierungs-Tour**: Die Scheibe wird nacheinander an 6 Rasterpunkten platziert. Der Arm parkt bei `(-150, 150)`, um die Sicht der Deckenkamera freizugeben.
  4. **Detektion**: Die Deckenkamera detektiert die Scheibe mittels Kreisschwellenwertbildung (`detect_object_cam2`).
  5. **Reprojektion**: Es wird eine affine 2D-Transformationsmatrix (Pixel zu Roboter-X/Y) mittels Least-Squares-Lösung berechnet.
- **Fehlertoleranzen & Optimierungen**:
  - Die Filterbeschränkung in `detect_object_cam2` wurde auf `cY >= 10` abgesenkt, um Scheibenpositionen am oberen Bildrand zu erfassen.
  - Wenn ein Re-Pick-Schritt fehlschlägt, bricht die Kalibrierungstour ab, um Kollisionen durch Fehlpositionierung zu verhindern.

### 2. Motion-Difference Kalibrierung ([calibrate_overhead_motion.py](file:///home/jpw/ai-projects/robot-arm-video-feedback/calibrate_overhead_motion.py))
Diese Methode kalibriert die Deckenkamera ohne physische Interaktion mit Objekten, allein über die Detektion der Eigenbewegung des Arm-Effektors.

- **Konzept**: Da der Effektor das einzige sich bewegende Element im Videobild ist, kann seine Position im Pixelraum über die zeitliche Differenz zweier Frames isoliert werden.
- **Ablauf**:
  1. Der Roboter führt eine Referenzfahrt durch.
  2. Er steuert nacheinander 3 Kalibrierpunkte bei `Z = -30.0` an.
  3. An jedem Punkt wird ein Frame (`f1`) aufgenommen, der Arm um `25.0` mm in Y-Richtung verschoben und ein zweiter Frame (`f2`) aufgenommen.
  4. Die Differenz `cv2.absdiff(gray1, gray2)` wird berechnet und schwellenwertbasiert segmentiert.
  5. Der Momenten-Schwerpunkt der größten Bewegungs-Kontur liefert den Bewegungsmittelpunkt (Greiferspitze) in Pixelkoordinaten.
  6. Aus den 3 Punktepaaren wird über `cv2.getAffineTransform` die Kalibrierungsmatrix berechnet.
- **Vorteil**: Mechanisch robust und verschleißfrei, da kein Ansaugen oder präzises Ablegen eines Objekts erforderlich ist.

### 3. 3D-Gitter-Sweeps ([calibrate_3d_grid.py](file:///home/jpw/ai-projects/robot-arm-video-feedback/calibrate_3d_grid.py))
Diese Methode dient zur Generierung von 3D-zu-2D-Projektionsdaten für die Deckenkamera (`/dev/video2`) und die Seitenkamera (`/dev/video0`).

- **Konzept**: Es wird ein systematisches 3D-Raumgitter (3x3x2 = 18 Punkte) abgefahren, um die Verzeichnung und geometrische Zuordnung im gesamten Arbeitsraum zu vermessen.
- **Ablauf**:
  1. Der Arm parkt bei `(0, 310, 80)`, um leere Hintergrundbilder der Decken- und Seitenkamera aufzunehmen.
  2. Der Arm fährt ein 3D-Raster ab (`X: [-100, 0, 100]`, `Y: [180, 230, 280]`, `Z: [0, 40]`).
  3. An jedem Punkt detektiert ein Differenzbild-Algorithmus (`find_gripper_point`) den tiefsten Punkt der Differenzkontur (Greiferspitze).
  4. Die Paare aus Roboter-Sollkoordinaten `(X, Y, Z)` und den jeweiligen Pixelkoordinaten `(u, v)_top` und `(u, v)_side` werden in [grid_calibration_data.npy](file:///home/jpw/ai-projects/robot-arm-video-feedback/calib_results/grid_calibration_data.npy) gespeichert.
- **Verwendung**: Diese Daten dienen nachfolgenden 3D-Stereorekonstruktionen oder nicht-linearen Optimierungen des Kamerasystems.

### 4. Automatische BFS-Arbeitsraum-Kartierung ([calibrate_grid_aruco.py](file:///home/jpw/ai-projects/robot-arm-video-feedback/calibrate_grid_aruco.py))
Dieses Skript führt eine automatische, hochauflösende Vermessung des gesamten von oben sichtbaren Arbeitsbereichs mittels eines ArUco-Markers (ID `0`) durch.

- **Konzept**: Der Arm startet direkt auf dem ArUco-Marker. Mittels eines Breadth-First-Search (BFS) Algorithmus wird das Gitter mit einer Auflösung von `1cm` (10mm) in X- und Y-Richtung radial nach außen hin vermessen.
- **Ablauf**:
  1. **Startzustand**: Der Sauger wird manuell auf dem ArUco-Marker positioniert. Das Skript liest diese Koordinate als Ursprung `(X_start, Y_start, Z_start)` aus.
  2. **Homing & Nullpunkt**: Der Arm hebt ab, führt ein Homing (`M1008 A5`) durch, parkt bei `(-150, 150)` und erfasst die Startpixelkoordinate `(u_start, v_start)` des Markers über die Deckenkamera.
  3. **Greifen**: Der Arm fährt zurück, saugt das Objekt an und hebt es auf Transporthöhe an.
  4. **BFS-Raster-Exploration**:
     - Das Skript holt die nächsten Koordinaten `(i, j)` (entspricht `i * 10mm`, `j * 10mm` Versatz) aus der BFS-Queue.
     - Es prüft die Erreichbarkeit mittels `checkXYZ`.
     - Es fährt den Zielpunkt an, setzt das Objekt ab, parkt den Arm und erfasst ein Bild.
     - Wenn der ArUco-Marker `0` detektiert wird, wird der Punkt gespeichert und seine Nachbarn `(i±1, j±1)` werden für die weitere Exploration vorgemerkt.
     - Wird der Marker nicht mehr im Kamerabild gefunden oder ist die Koordinate mechanisch unerreichbar, wird dieser Ast abgebrochen (die Suche geht in dieser Richtung nicht weiter).
     - Der Arm fährt zurück, greift das Objekt wieder und holt es ab.
  5. **Speicherung**: Am Ende wird die Zuordnungstabelle zwischen Pixelkoordinaten und physischen Armkoordinaten als JSON-Datei unter [aruco_grid_mapping.json](file:///home/jpw/ai-projects/robot-arm-video-feedback/calib_results/aruco_grid_mapping.json) gespeichert.

---

## Inbetriebnahme & Ausführung

1. Installieren Sie die Python-Abhängigkeiten im aktiven Virtual Environment:
   ```bash
   pip install -r requirements.txt
   ```
2. Stellen Sie sicher, dass alle drei USB-Kameras an den Ports `/dev/video0`, `/dev/video2` und `/dev/video6` gemountet sind.
3. Starten Sie das gewünschte Kalibrierungsskript, zum Beispiel:
   ```bash
   python calibrate_overhead_motion.py
   ```
   Die berechneten affinen Matrizen und Ergebnisbilder werden im Unterverzeichnis `./calib_results` abgelegt.
