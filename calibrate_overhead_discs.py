#!/usr/bin/env python3
import sys
import os
import time
import cv2
import numpy as np
import threading

# Import local robot_control library
from robot_control.robot import openSerial, checkXYZ

# Configuration
CALIBRATION_POINTS = [
    (  0.0, 200.0),
    ( 80.0, 200.0),
    (-80.0, 200.0),
    (  0.0, 240.0),
    ( 80.0, 240.0),
    (-80.0, 240.0),
]

Z_TRANSPORT = 40.0
Z_SEARCH    = 106.5   # Hand camera tracking height
Z_PICK      = -45.0   # Grip depth (adjusted for 1cm uniform base)
Z_PLACE     = -42.7   # Place depth (adjusted for 1cm uniform base)
PARK_X, PARK_Y = -150.0, 150.0

# Hand camera visual servoing gains (at Z=106.5mm)
KP_X_U, KP_X_V = -0.0285, -0.0009
KP_Y_U, KP_Y_V = -0.0003, +0.0400

TARGET_U, TARGET_V = 320, 240
STABLE_THRESH_PX = 20  # Pixel tolerance for centering stable state

class CameraReader:
    def __init__(self, dev_id):
        self.cap = cv2.VideoCapture(dev_id)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {dev_id}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.frame = None; self.ret = False; self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame; self.ret = True
            time.sleep(0.01)

    def read(self): return self.ret, self.frame

    def stop(self):
        self.running = False; self.thread.join(timeout=1.0); self.cap.release()

def capture_overhead(dev_id=2):
    cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened(): return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(30): cap.read() # Warm-up/autofocus flush
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None

def detect_object_cam2(frame):
    if frame is None: return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    candidates = {}
    for th in range(160, 225, 5):
        _, thresh = cv2.threshold(gray, th, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 1500 < area < 10000:
                peri = cv2.arcLength(cnt, True)
                if peri == 0: continue
                circ = 4 * np.pi * area / (peri * peri)
                if circ > 0.60:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        found = False
                        for k in candidates:
                            if abs(cX - k[0]) < 15 and abs(cY - k[1]) < 15:
                                if th > candidates[k][0]: candidates[k] = (th, circ)
                                found = True; break
                        if not found:
                            candidates[(cX, cY)] = (th, circ)
    # Lowered threshold to >= 10 to include coordinates high up in the frame
    filtered = {k: v for k, v in candidates.items() if k[1] >= 10}
    if not filtered: return None
    best = max(filtered.items(), key=lambda x: (x[1][1], x[1][0]))
    return best[0][0], best[0][1]

def find_black_dot(frame):
    if frame is None: return None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    max_b = np.max(gray)
    if max_b < 140: return None
    th_white = max(145, int(max_b * 0.88))
    _, thresh = cv2.threshold(gray, th_white, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt, best_area = None, 0
    for c in contours:
        area = cv2.contourArea(c)
        if 3000 < area < 90000:
            p = cv2.arcLength(c, True)
            circ = 4 * np.pi * area / (p * p) if p > 0 else 0
            if circ > 0.45 and area > best_area:
                best_area = area; best_cnt = c
    if best_cnt is None:
        for c in contours:
            area = cv2.contourArea(c)
            if area > 2500 and area > best_area:
                best_area = area; best_cnt = c
    if best_cnt is None: return None
    x, y, w, h = cv2.boundingRect(best_cnt)
    m_w, m_h = int(w * 0.32), int(h * 0.32)
    roi = gray[max(0, y+m_h):min(gray.shape[0], y+h-m_h),
               max(0, x+m_w):min(gray.shape[1], x+w-m_w)]
    if roi.size == 0: return None
    mn, mx, mn_loc, _ = cv2.minMaxLoc(roi)
    if mn > 135 or (mx - mn) < 55: return None
    return int(x + m_w + mn_loc[0]), int(y + m_h + mn_loc[1])

def move_and_wait(ser, x, y, z, f=3000, wait=3.0):
    if not checkXYZ(x, y, z):
        print(f"[WARN] ({x:.1f},{y:.1f},{z:.1f}) unreachable – skipping")
        return False
    cmd = f"G1 X{x:.2f} Y{y:.2f} Z{z:.2f} F{f}\nM400\n"
    ser.write(cmd.encode())
    time.sleep(wait)
    return True

def vacuum_on(ser):
    ser.write(b"M1401 A0\nM1400 A1023\n")
    time.sleep(0.5)

def vacuum_off(ser):
    ser.write(b"M1400 A0\nM1401 A1\n")
    time.sleep(1.5)
    ser.write(b"M1401 A0\n")

def avg_pos(reader, n=12, timeout=4.0):
    pts = []; t0 = time.time()
    while len(pts) < n and time.time() - t0 < timeout:
        ret, f = reader.read()
        if ret and f is not None:
            d = find_black_dot(f)
            if d: pts.append(d)
        time.sleep(0.04)
    if len(pts) < 3: return None
    return tuple(np.mean(pts, axis=0))

def fine_track_and_pick(ser, reader, x_start, y_start, out_dir):
    x_curr, y_curr = x_start, y_start
    stable_start = None
    last_cmd = time.time()
    t0 = time.time()

    while time.time() - t0 < 45.0:
        ret, frame = reader.read()
        if not ret or frame is None:
            time.sleep(0.01); continue
        dot = find_black_dot(frame)
        disp = frame.copy()
        if dot:
            u, v = dot
            err_u, err_v = TARGET_U - u, TARGET_V - v
            cv2.circle(disp, (u, v), 6, (0,255,0), -1)
            cv2.circle(disp, (TARGET_U, TARGET_V), 10, (255,0,0), 2)
            cv2.line(disp, (u,v), (TARGET_U,TARGET_V), (0,0,255), 2)

            if abs(err_u) <= STABLE_THRESH_PX and abs(err_v) <= STABLE_THRESH_PX:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= 2.0:
                    print(f"\n[OK] Stable at ({x_curr:.1f},{y_curr:.1f}). Picking up...")
                    return x_curr, y_curr
            else:
                stable_start = None

            now = time.time()
            if now - last_cmd >= 0.20:
                dx = KP_X_U * err_u + KP_X_V * err_v
                dy = KP_Y_U * err_u + KP_Y_V * err_v
                dx = np.clip(dx, -6.0, 6.0)
                dy = np.clip(dy, -6.0, 6.0)
                if abs(err_u) <= STABLE_THRESH_PX and abs(err_v) <= STABLE_THRESH_PX:
                    dx = dy = 0.0
                if dx or dy:
                    xn = np.clip(x_curr + dx, -220.0, 220.0)
                    yn = np.clip(y_curr + dy, 130.0, 290.0)
                    if checkXYZ(xn, yn, Z_SEARCH):
                        x_curr, y_curr = xn, yn
                        ser.write(f"G1 X{x_curr:.2f} Y{y_curr:.2f} Z{Z_SEARCH:.2f} F5000\n".encode())
                        if ser.in_waiting: ser.read_all()
                        last_cmd = now
                        print(f"[TRACK] ({u},{v}) err=({err_u},{err_v}) pos=({x_curr:.1f},{y_curr:.1f})", end="\r", flush=True)
        else:
            stable_start = None
        cv2.imwrite(os.path.join(out_dir, "calib_view.jpg"), disp)
        time.sleep(0.01)

    print("\n[WARN] Fine tracking timed out")
    return None

def run_calibration():
    out_dir = "./calib_results"
    os.makedirs(out_dir, exist_ok=True)

    print("Opening serial connection...")
    try:
        ser = openSerial()
        ser.timeout = 0.5; ser.rts = False
    except Exception as e:
        print(f"[ERROR] {e}"); return

    # Enable motor, G90 absolute coords, and home robot arm
    ser.write(b"M17\nG90\nM1008 A5\n")
    time.sleep(8.0)  # Wait for homing

    print("[INIT] Opening hand camera /dev/video6 (2s warm-up)...")
    cam = CameraReader(6)
    time.sleep(2.0)

    # Search grid for locating disc
    SEARCH_GRID = [
        (  0.0, 200.0), (-11.87, 239.10),
        (  0.0, 220.0), (  0.0, 180.0),
        (-60.0, 200.0), (-60.0, 220.0), (-60.0, 180.0),
        (-120.0,200.0), (-120.0,220.0), (-120.0,240.0),
        (-150.0,200.0), (-150.0,240.0),
        ( 60.0, 200.0), ( 60.0, 220.0), ( 60.0, 180.0),
        (120.0, 200.0), (120.0, 220.0), (120.0, 240.0),
        (150.0, 200.0), (150.0, 240.0),
    ]

    start_x, start_y = None, None
    print("\n[STEP 0] Suchraster – Handkamera sucht Scheibe...")
    for sx, sy in SEARCH_GRID:
        if not checkXYZ(sx, sy, Z_SEARCH):
            continue
        move_and_wait(ser, sx, sy, Z_SEARCH, wait=2.0)
        p = avg_pos(cam, n=8, timeout=2.5)
        if p is not None:
            start_x, start_y = sx, sy
            print(f"[FOUND] Scheibe sichtbar bei Arm({sx:.1f},{sy:.1f}) -> Pixel({p[0]:.1f},{p[1]:.1f})")
            break
        else:
            print(f"  kein Fund bei ({sx:.0f},{sy:.0f})", end="\r")

    if start_x is None:
        print("\n[ERROR] Scheibe in keiner Suchposition gefunden. Abbruch.")
        cam.stop(); ser.close(); return

    result = fine_track_and_pick(ser, cam, start_x, start_y, out_dir)
    if result is None:
        print("[ERROR] Fein-Tracking fehlgeschlagen.")
        cam.stop(); ser.close(); return

    x_pick, y_pick = result
    vacuum_on(ser)
    move_and_wait(ser, x_pick, y_pick, Z_PICK, f=2000, wait=4.0)
    time.sleep(2.0)
    move_and_wait(ser, x_pick, y_pick, Z_TRANSPORT, f=3000, wait=3.0)
    print(f"[OK] Disc picked at ({x_pick:.1f},{y_pick:.1f})")

    calib_data = []

    for idx, (cal_x, cal_y) in enumerate(CALIBRATION_POINTS):
        print(f"\n[CALIB {idx+1}/{len(CALIBRATION_POINTS)}] Target: X={cal_x:.1f} Y={cal_y:.1f}")

        # Transport and place
        if not move_and_wait(ser, cal_x, cal_y, Z_TRANSPORT, wait=3.0):
            print("[SKIP] Unreachable"); continue

        move_and_wait(ser, cal_x, cal_y, Z_PLACE, f=1500, wait=3.0)
        vacuum_off(ser)
        move_and_wait(ser, cal_x, cal_y, Z_TRANSPORT, f=3000, wait=2.0)

        # Park arm to clear overhead camera view
        move_and_wait(ser, PARK_X, PARK_Y, Z_TRANSPORT, f=4000, wait=3.0)

        # Capture and detect from overhead camera (/dev/video2)
        frame = capture_overhead(2)
        res = detect_object_cam2(frame)

        if res is None:
            print(f"[WARN] Disc not detected in overhead image at calib point {idx+1}")
            if frame is not None:
                cv2.imwrite(os.path.join(out_dir, f"calib_{idx+1}_miss.jpg"), frame)
            # Try to re-pick anyway using last known position
            move_and_wait(ser, cal_x, cal_y, Z_SEARCH, wait=4.0)
            r = fine_track_and_pick(ser, cam, cal_x, cal_y, out_dir)
            if r:
                x_pick, y_pick = r
                vacuum_on(ser)
                move_and_wait(ser, x_pick, y_pick, Z_PICK, f=2000, wait=4.0)
                time.sleep(2.0)
                move_and_wait(ser, x_pick, y_pick, Z_TRANSPORT, f=3000, wait=3.0)
            else:
                print("[ERROR] Re-pick failed. Stopping calibration.")
                break
            continue

        u_det, v_det = res
        print(f"[DETECT] Pixel ({u_det}, {v_det}) at robot ({cal_x:.1f}, {cal_y:.1f})")
        calib_data.append((cal_x, cal_y, u_det, v_det))

        if frame is not None:
            disp = frame.copy()
            cv2.circle(disp, (u_det, v_det), 8, (0, 0, 255), -1)
            cv2.putText(disp, f"X={cal_x:.0f} Y={cal_y:.0f}", (u_det+10, v_det),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(out_dir, f"calib_{idx+1}_X{int(cal_x)}_Y{int(cal_y)}.jpg"), disp)

        # Re-pick disc
        move_and_wait(ser, cal_x, cal_y, Z_SEARCH, wait=4.0)
        r = fine_track_and_pick(ser, cam, cal_x, cal_y, out_dir)
        if r:
            x_pick, y_pick = r
            vacuum_on(ser)
            move_and_wait(ser, x_pick, y_pick, Z_PICK, f=2000, wait=4.0)
            time.sleep(2.0)
            move_and_wait(ser, x_pick, y_pick, Z_TRANSPORT, f=3000, wait=3.0)
            print(f"[OK] Re-picked at ({x_pick:.1f},{y_pick:.1f})")
        else:
            print("[ERROR] Re-pick failed. Stopping calibration.")
            break

    # Compute affine matrix
    print(f"\n[RESULT] Collected {len(calib_data)} calibration points")
    if len(calib_data) >= 3:
        A = np.array([[cd[2], cd[3], 1.0] for cd in calib_data], dtype=np.float64)
        bx = np.array([cd[0] for cd in calib_data], dtype=np.float64)
        by = np.array([cd[1] for cd in calib_data], dtype=np.float64)

        row_x, _, _, _ = np.linalg.lstsq(A, bx, rcond=None)
        row_y, _, _, _ = np.linalg.lstsq(A, by, rcond=None)

        M_new = np.array([row_x, row_y])
        print(f"\n[NEW AFFINE MATRIX]")
        print(f"M_affine = np.array([")
        print(f"    [{M_new[0,0]:.6f}, {M_new[0,1]:.6f}, {M_new[0,2]:.6f}],")
        print(f"    [{M_new[1,0]:.6f}, {M_new[1,1]:.6f}, {M_new[1,2]:.6f}]")
        print(f"])")

        np.save(os.path.join(out_dir, "M_affine_overhead.npy"), M_new)
        print(f"\n[SAVED] Matrix written to {out_dir}/M_affine_overhead.npy")
    else:
        print("[ERROR] Fewer than 3 calibration points – cannot compute matrix")

    # Place disc at home and clean up
    print("\n[CLEANUP] Placing disc at home position...")
    home_x, home_y = -11.87, 239.10
    move_and_wait(ser, home_x, home_y, Z_TRANSPORT, wait=3.0)
    move_and_wait(ser, home_x, home_y, Z_PLACE, f=1500, wait=3.0)
    vacuum_off(ser)
    move_and_wait(ser, home_x, home_y, Z_TRANSPORT, wait=2.0)

    cam.stop()
    ser.close()
    print("[DONE] Calibration finished.")

if __name__ == "__main__":
    run_calibration()
