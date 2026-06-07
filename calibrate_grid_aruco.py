#!/usr/bin/env python3
import sys
import os
import time
import cv2
import numpy as np
import json
from collections import deque

# Import local robot_control library
from robot_control.robot import openSerial, checkXYZ, getLoc

def send_gcode(ser, cmd, timeout=15.0):
    lines = [l.strip() for l in cmd.split("\n") if l.strip()]
    for line in lines:
        ser.write((line + "\n").encode())
        start_time = time.time()
        ok_found = False
        while time.time() - start_time < timeout:
            ret_line = ser.readline().decode("utf-8", errors="ignore")
            if "ok" in ret_line:
                ok_found = True
                break
        if not ok_found:
            print(f"  [WARN] Timeout beim Warten auf 'ok' fuer Befehl: {line}")

def capture_frame(cap, flush_frames=8):
    if cap is None or not cap.isOpened():
        return None
    # Flush older frames in driver buffer
    for _ in range(flush_frames):
        cap.read()
    ret, frame = cap.read()
    return frame if ret else None

def detect_aruco_marker_0(frame):
    if frame is None:
        return None
    
    dicts = [
        cv2.aruco.DICT_4X4_50,
        cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_6X6_50,
        cv2.aruco.DICT_APRILTAG_36h11
    ]
    
    for d_id in dicts:
        dictionary = cv2.aruco.getPredefinedDictionary(d_id)
        try:
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejected = detector.detectMarkers(frame)
        except AttributeError:
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 0:
                    pts = corners[i][0]
                    cX = int(np.mean(pts[:, 0]))
                    cY = int(np.mean(pts[:, 1]))
                    return cX, cY
    return None

def fine_adjust_pick(ser, cap_hand, rx, ry, Z_verify, U_target, V_target, max_steps=5):
    # Justiert den Arm ueber (rx, ry) auf Z_verify, bis der Marker 
    # bei (U_target, V_target) zentriert ist.
    print(f"  [JUSTIERUNG] Starte Feinjustierung bei Soll-Koordinate X={rx:.1f}, Y={ry:.1f}...")
    x_curr, y_curr = rx, ry
    
    for step in range(1, max_steps + 1):
        # Fahre zu aktueller Position
        send_gcode(ser, f"G1 X{x_curr:.2f} Y{y_curr:.2f} Z{Z_verify:.2f} F5000\nM400")
        
        # Soll-Ist-Abgleich der Fahrt
        act_pos = getLoc()
        if not act_pos or len(act_pos) < 3 or abs(act_pos[0] - x_curr) > 1.0 or abs(act_pos[1] - y_curr) > 1.0:
            print(f"    [WARN] Justierung: Ziel-Koordinate X={x_curr:.1f}, Y={y_curr:.1f} nicht erreichbar!")
            return None, None
            
        time.sleep(0.2) # Beruhigung
        
        # Bild machen
        frame = capture_frame(cap_hand, flush_frames=3)
        p_marker = detect_aruco_marker_0(frame)
        
        if p_marker is None:
            print(f"    [WARN] Justierung Schritt {step}: Marker von Handkamera nicht detektiert!")
            return None, None
            
        u, v = p_marker
        err_u = u - U_target
        err_v = v - V_target
        dist_px = np.sqrt(err_u**2 + err_v**2)
        
        print(f"    Schritt {step}: u={u}, v={v} -> Fehler err_u={err_u:.1f}, err_v={err_v:.1f} (Dist={dist_px:.1f}px)")
        
        if dist_px < 4.0:
            print(f"    [JUSTIERUNG] Toleranz erreicht (< 4.0px). Zentrierte Position: X={x_curr:.2f}, Y={y_curr:.2f}")
            return x_curr, y_curr
            
        # P-Regler: 1 Pixel entspricht ca. 0.15 mm bei Z=77.3
        dx = -0.12 * err_u
        dy = 0.12 * err_v
        
        # Schrittbegrenzung auf 5mm zur Sicherheit
        dx = np.clip(dx, -5.0, 5.0)
        dy = np.clip(dy, -5.0, 5.0)
        
        x_curr += dx
        y_curr += dy
        
        if not checkXYZ(x_curr, y_curr, Z_verify):
            print("    [WARN] Justierungs-Schritt verlaesst Roboter-Arbeitsraum!")
            return None, None
            
    print(f"  [WARN] Feinjustierung: Maximale Schritte ({max_steps}) ohne Konvergenz erreicht.")
    return x_curr, y_curr

def pick_object_with_retry(ser, cap_top, cap_hand, rx, ry, Z_pick, Z_verify, Z_travel, U_target, V_target, PARK_X, PARK_Y, max_retries=3):
    for attempt in range(1, max_retries + 1):
        print(f"  Abhol-Versuch {attempt}/{max_retries}...")
        
        # 1. Feinjustierung auf Z_verify durchfuehren mit Handkamera
        x_just, y_just = fine_adjust_pick(ser, cap_hand, rx, ry, Z_verify, U_target, V_target)
        if x_just is None or y_just is None:
            print("  -> Feinjustierung fehlgeschlagen. Versuche direkten Pick mit Sollkoordinaten...")
            x_just, y_just = rx, ry
            
        # 2. Absenken auf Z_pick
        send_gcode(ser, f"G1 X{x_just:.2f} Y{y_just:.2f} Z{Z_pick:.2f} F4000\nM400")
        
        # Soll-Ist-Abgleich beim Absenken
        act_pos = getLoc()
        if not act_pos or len(act_pos) < 3 or abs(act_pos[2] - Z_pick) > 1.0:
            print(f"  -> [WARN] Absenken auf Z_pick ({Z_pick:.1f}) blockiert! Ist-Z: {act_pos[2] if act_pos else 'None'}. Abbruch.")
            send_gcode(ser, f"G1 Z{Z_verify:.2f} F4000\nM400")
            continue
            
        # 3. Vakuum an (0.5s build-up)
        send_gcode(ser, "M1401 A0\nM1400 A1023")
        time.sleep(0.5)
        
        # 4. Anheben auf Z_travel (kein Ausschalten in 10cm Hoehe)
        send_gcode(ser, f"G1 Z{Z_travel:.2f} F4000\nM400")
        
        # 5. In Parkposition fahren zur Verifikation
        send_gcode(ser, f"G1 X{PARK_X:.2f} Y{PARK_Y:.2f} Z{Z_travel:.2f} F8000\nM400")
        time.sleep(0.5) # Beruhigungszeit fuer Deckenkamera
        
        # 6. Verifikation mit Deckenkamera (cap_top)
        frame = capture_frame(cap_top, flush_frames=5)
        p_marker = detect_aruco_marker_0(frame)
        
        if p_marker is None:
            print("  -> Abholen erfolgreich (Deckenkamera sieht das Objekt nicht mehr auf dem Tisch).")
            return True
        else:
            print(f"  -> [WARN] Objekt nicht aufgehoben (Deckenkamera detektiert Marker noch auf dem Tisch bei Pixel {p_marker}).")
            # Vakuum aus (nur Pumpe aus, kein Luftstoss, um das Objekt nicht zu verschieben)
            send_gcode(ser, "M1400 A0")
            time.sleep(0.5) # Erholungszeit
            
    print("[ERROR] Objekt konnte nach 3 Versuchen nicht aufgehoben werden.")
    return False

def main():
    out_dir = "./calib_results"
    debug_dir = os.path.join(out_dir, "aruco_grid_debug")
    os.makedirs(debug_dir, exist_ok=True)
    
    print("=== ArUco Gitter-Kalibrierung (Optimiert) ===")
    
    try:
        ser = openSerial()
        ser.timeout = 2.0; ser.rts = False
    except Exception as e:
        print(f"[ERROR] Serial open failed: {e}")
        return
        
    start_pos = getLoc()
    if not start_pos or len(start_pos) < 3:
        print("[ERROR] Konnte aktuelle Arm-Position nicht auslesen.")
        ser.close()
        return
        
    X_start, Y_start, Z_start = start_pos[0], start_pos[1], start_pos[2]
    print(f"Startposition (Aruco-Marker 0): X={X_start:.2f}, Y={Y_start:.2f}, Z={Z_start:.2f}")
    
    Z_place = Z_start + 1.0  # Freigabe aus 1mm Hoehe
    Z_pick = Z_start - 4.5   # 4.5mm Kompression fuer Pick
    Z_verify = 77.3          # Optimal hand camera detection height
    Z_travel = Z_start + 40.0
    PARK_X, PARK_Y = -150.0, 150.0
    
    # Kameras initialisieren
    print("[INIT] Oeffne Overhead-Kamera (/dev/video2)...")
    cap_top = cv2.VideoCapture(2)
    if not cap_top.isOpened():
        print("[ERROR] Konnte Deckenkamera nicht oeffnen.")
        ser.close()
        return
    cap_top.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap_top.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_top.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap_top.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    print("[INIT] Oeffne Handkamera (/dev/video6)...")
    cap_hand = cv2.VideoCapture(6)
    if not cap_hand.isOpened():
        print("[ERROR] Konnte Handkamera nicht oeffnen.")
        cap_top.release()
        ser.close()
        return
    cap_hand.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap_hand.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_hand.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_hand.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Warmup
    for _ in range(15):
        cap_top.read()
        cap_hand.read()
        
    grid_data = {}
    GRID_STEP = 30.0  # Gitterweite in mm (3.0 cm)
    mapping_path = os.path.join(out_dir, "aruco_grid_mapping.json")
    
    try:
        # Homing durchfuehren
        print("\nSchritt 2: Roboterarm anheben und Homing durchführen...")
        send_gcode(ser, f"G1 Z{Z_travel:.2f} F4000\nM400")
        send_gcode(ser, "M1008 A5")
        time.sleep(1.0)
        send_gcode(ser, "M17\nG90")
        
        # Nullpunkt-Justierung fuer Handkamera (Sollwert bestimmen)
        print("\nFahre auf Referenzhoehe zur Handkamera-Referenzierung...")
        send_gcode(ser, f"G1 X{X_start:.2f} Y{Y_start:.2f} Z{Z_verify:.2f} F5000\nM400")
        time.sleep(0.5)
        
        frame_ref = capture_frame(cap_hand, flush_frames=5)
        p_ref = detect_aruco_marker_0(frame_ref)
        if p_ref is None:
            print("[ERROR] Konnte ArUco-Marker 0 auf Referenzhoehe nicht fuer Handkamera kalibrieren.")
            return
            
        U_target, V_target = p_ref
        print(f"[OK] Handkamera-Referenzpunkt eingemessen bei Pixel: ({U_target}, {V_target})")
        
        # Parken zur Erfassung der Deckenkamera-Initialposition
        print(f"Fahre in Parkposition (X={PARK_X}, Y={PARK_Y}) zur freien Sicht...")
        send_gcode(ser, f"G1 X{PARK_X:.2f} Y{PARK_Y:.2f} Z{Z_travel:.2f} F8000\nM400")
        time.sleep(0.5)
        
        f_init = capture_frame(cap_top, flush_frames=5)
        p_init = detect_aruco_marker_0(f_init)
        if p_init is None:
            print("[ERROR] Aruco-Marker 0 konnte an der Initialposition von der Deckenkamera nicht detektiert werden.")
            return
            
        u_start, v_start = p_init
        print(f"Aruco-Marker 0 auf Deckenkamera gefunden bei Pixel: ({u_start}, {v_start})")
        
        grid_data["0,0"] = {
            "robot": [X_start, Y_start],
            "pixel": [u_start, v_start]
        }
        with open(mapping_path, "w") as f:
            json.dump(grid_data, f, indent=4)
            
        if f_init is not None:
            cv2.circle(f_init, p_init, 8, (0, 255, 0), -1)
            cv2.imwrite(os.path.join(debug_dir, "grid_0_0.jpg"), f_init)
            
        # Initialer Pick
        print("Hole Objekt von Initialposition ab...")
        success = pick_object_with_retry(ser, cap_top, cap_hand, X_start, Y_start, Z_pick, Z_verify, Z_travel, U_target, V_target, PARK_X, PARK_Y, max_retries=3)
        if not success:
            print("[ERROR] Initialer Pick fehlgeschlagen. Abbruch.")
            return
            
        # BFS Grid Search
        Q = deque([(1, 0), (-1, 0), (0, 1), (0, -1)])
        visited = {(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)}
        
        print(f"\nSchritt 3: Starte automatische BFS-Gitter-Kalibrierung ({GRID_STEP/10.0:.1f}cm Auflösung)...")
        
        while Q:
            i, j = Q.popleft()
            rx = X_start + i * GRID_STEP
            ry = Y_start + j * GRID_STEP
            
            # Workspace limits check
            if not checkXYZ(rx, ry, Z_place) or not checkXYZ(rx, ry, Z_travel):
                print(f"  Raster ({i},{j}) -> Roboter ({rx:.1f}, {ry:.1f}) außerhalb Arbeitsraum.")
                continue
                
            print(f"\nTeste Raster ({i},{j}) -> Fahre zu X={rx:.1f}, Y={ry:.1f}...")
            
            # 1. Fahre ueber das Ziel auf Z_travel
            send_gcode(ser, f"G1 X{rx:.2f} Y{ry:.2f} Z{Z_travel:.2f} F8000\nM400")
            act_pos = getLoc()
            if not act_pos or len(act_pos) < 3 or abs(act_pos[0] - rx) > 1.0 or abs(act_pos[1] - ry) > 1.0:
                print(f"  [WARN] Soll-Ist-Abweichung zu gross auf Z_travel. Soll X,Y: ({rx:.1f}, {ry:.1f}), Ist: ({act_pos[0]:.1f}, {act_pos[1]:.1f}). Ueberspringe.")
                continue
                
            # 2. Senke ab auf Z_place
            send_gcode(ser, f"G1 Z{Z_place:.2f} F4000\nM400")
            act_pos = getLoc()
            if not act_pos or len(act_pos) < 3 or abs(act_pos[2] - Z_place) > 1.0:
                print(f"  [WARN] Soll-Ist-Abweichung zu gross auf Z_place. Ist-Z: {act_pos[2] if act_pos else 'None'} vs Soll: {Z_place:.1f}. Fahre hoch und ueberspringe.")
                send_gcode(ser, f"G1 Z{Z_travel:.2f} F4000\nM400")
                continue
                
            # 3. Vakuum aus
            send_gcode(ser, "M1400 A0\nM1401 A1")
            time.sleep(1.5)
            send_gcode(ser, "M1401 A0")
            
            # 4. Hochfahren
            send_gcode(ser, f"G1 Z{Z_travel:.2f} F4000\nM400")
            
            # 5. Parken zur Erfassung mit Deckenkamera
            send_gcode(ser, f"G1 X{PARK_X:.2f} Y{PARK_Y:.2f} Z{Z_travel:.2f} F8000\nM400")
            time.sleep(0.5)
            
            # 6. Capture und Detect (Overhead)
            frame = capture_frame(cap_top, flush_frames=3)
            p_marker = detect_aruco_marker_0(frame)
            
            if p_marker is not None:
                u, v = p_marker
                # Verify movement
                if len(grid_data) > 0:
                    last_key = list(grid_data.keys())[-1]
                    last_pixel = grid_data[last_key]["pixel"]
                    dist = np.sqrt((u - last_pixel[0])**2 + (v - last_pixel[1])**2)
                    if dist < 4.0:
                        print(f"  [ERROR] Objekt hat sich nicht bewegt (liegt immer noch bei Pixel {last_pixel}). Abholen war fehlerhaft! Abbrechen.")
                        break
                        
                print(f"  -> Detektiert bei Pixel ({u}, {v})")
                grid_data[f"{i},{j}"] = {
                    "robot": [rx, ry],
                    "pixel": [u, v]
                }
                with open(mapping_path, "w") as f:
                    json.dump(grid_data, f, indent=4)
                    
                if frame is not None:
                    cv2.circle(frame, p_marker, 8, (0, 255, 0), -1)
                    cv2.imwrite(os.path.join(debug_dir, f"grid_{i}_{j}.jpg"), frame)
                    
                # Nachbarn hinzufuegen
                neighbors = [(i+1, j), (i-1, j), (i, j+1), (i, j-1)]
                for n in neighbors:
                    if n not in visited:
                        visited.add(n)
                        Q.append(n)
            else:
                print("  -> Marker NICHT detektiert (außerhalb des Deckenkamera-Sichtbereichs). Ast abgebrochen.")
                if frame is not None:
                    cv2.imwrite(os.path.join(debug_dir, f"grid_{i}_{j}_miss.jpg"), frame)
                    
            # 7. Re-pick mit Deckenkamera-Verifikation
            success = pick_object_with_retry(ser, cap_top, cap_hand, rx, ry, Z_pick, Z_verify, Z_travel, U_target, V_target, PARK_X, PARK_Y, max_retries=3)
            if not success:
                print("[ERROR] Kalibrierung abgebrochen, da das Objekt nicht aufgehoben werden konnte.")
                break
                
    finally:
        # Am Ende zur Startposition zurueckfahren
        print("\nKalibrierung beendet. Fahre Objekt zurück zur Startposition...")
        try:
            # Versuche das Objekt an der Startposition abzusetzen, falls es noch am Sauger haengt
            # Verifikation per Handkamera (falls Marker nicht detektiert wird, haben wir das Objekt)
            frame_final = capture_frame(cap_hand, flush_frames=3)
            if detect_aruco_marker_0(frame_final) is None:
                # Wir haben das Objekt, also absetzen an Startposition
                send_gcode(ser, f"G1 X{X_start:.2f} Y{Y_start:.2f} Z{Z_travel:.2f} F8000\nM400")
                send_gcode(ser, f"G1 Z{Z_place:.2f} F4000\nM400")
                send_gcode(ser, "M1400 A0\nM1401 A1")
                time.sleep(1.5)
                send_gcode(ser, "M1401 A0")
                send_gcode(ser, f"G1 Z{Z_travel:.2f} F4000\nM400")
        except Exception as e:
            print(f"  [WARN] Zurueckfahren zur Startposition fehlgeschlagen: {e}")
            
        with open(mapping_path, "w") as f:
            json.dump(grid_data, f, indent=4)
            
        print(f"\n[SUCCESS] Gitter-Kalibrierung beendet.")
        print(f"Insgesamt {len(grid_data)} Punkte erfolgreich kartografiert.")
        print(f"Daten gespeichert unter: {mapping_path}")
        
        try:
            cap_top.release()
            cap_hand.release()
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
