#!/usr/bin/env python3
import cv2
import numpy as np
import os
import time
import sys

# Import local robot_control library
from robot_control.robot import openSerial, moveG0, checkXYZ

def capture_frame(dev_id):
    cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {dev_id}")
        return None
    # For top camera (/dev/video2) use MJPEG 1280x720, else standard
    if dev_id == 2:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    else:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    for _ in range(15):
        cap.read()
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None

def find_gripper_point(bg_gray, img, threshold_val=20):
    if img is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Calculate difference image
    diff = cv2.absdiff(gray, bg_gray)
    
    # Thresholding
    _, thresh = cv2.threshold(diff, threshold_val, 255, cv2.THRESH_BINARY)
    
    # Morphological cleaning
    kernel = np.ones((5,5), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
        
    # Take largest contour (should be the arm)
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 100: # Noise filter
        return None
        
    # The bottom-most point of the contour (maximum Y-coordinate in image space)
    # represents the gripper tip
    extBot = tuple(c[c[:, :, 1].argmax()][0])
    return (int(extBot[0]), int(extBot[1]))

def main():
    print("Starte 3D-Gitter-Kalibrierung (18 Punkte)...")
    out_dir = "./calib_results"
    os.makedirs(out_dir, exist_ok=True)
    
    try:
        ser = openSerial()
        ser.timeout = 2.0; ser.rts = False
    except Exception as e:
        print(f"[ERROR] Serial open failed: {e}")
        return
        
    ser.write(b"M17\nG90\nM1008 A5\n")
    time.sleep(8.0) # Wait for homing
    
    # 1. Capture background images at park position (arm fully out of view)
    park_x, park_y, park_z = 0.0, 310.0, 80.0
    print(f"Fahre in Parkposition: X={park_x}, Y={park_y}, Z={park_z}...")
    moveG0(park_x, park_y, park_z)
    time.sleep(4.0)
    
    print("Erfasse Hintergrundbilder...")
    bg_top_raw = capture_frame(2) # Top is dev 2
    bg_side_raw = capture_frame(0) # Side is dev 0
    
    if bg_top_raw is None or bg_side_raw is None:
        print("[ERROR] Failed to capture background frames.")
        ser.close()
        return
        
    bg_top_gray = cv2.cvtColor(bg_top_raw, cv2.COLOR_BGR2GRAY)
    bg_side_gray = cv2.cvtColor(bg_side_raw, cv2.COLOR_BGR2GRAY)
    
    cv2.imwrite(os.path.join(out_dir, "bg_top_cal.jpg"), bg_top_raw)
    cv2.imwrite(os.path.join(out_dir, "bg_side_cal.jpg"), bg_side_raw)
    
    # 2. Define grid points (3x3x2 = 18 points)
    x_coords = [-100.0, 0.0, 100.0]
    y_coords = [180.0, 230.0, 280.0]
    z_levels = [0.0, 40.0]
    
    robot_points = []
    for z in z_levels:
        for y in y_coords:
            for x in x_coords:
                robot_points.append((x, y, z))
                
    pixel_data = []
    
    # 3. Drive points and capture coordinates
    for i, (rx, ry, rz) in enumerate(robot_points):
        pt_idx = i + 1
        print(f"\n[{pt_idx}/18] Fahre zu: X={rx}, Y={ry}, Z={rz}...")
        if not checkXYZ(rx, ry, rz):
            print("  [SKIP] Target unreachable")
            continue
            
        moveG0(rx, ry, rz)
        time.sleep(4.0) # wait for settling
        
        f_top = capture_frame(2)
        f_side = capture_frame(0)
        
        p_top = find_gripper_point(bg_top_gray, f_top, threshold_val=20)
        p_side = find_gripper_point(bg_side_gray, f_side, threshold_val=20)
        
        if p_top is not None and p_side is not None:
            print(f"  -> Detektiert: Top={p_top}, Side={p_side}")
            pixel_data.append({
                "robot": (rx, ry, rz),
                "top": p_top,
                "side": p_side
            })
            
            # Save annotated debug images
            if f_top is not None:
                cv2.circle(f_top, p_top, 8, (0, 0, 255), -1)
                cv2.drawMarker(f_top, p_top, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.imwrite(os.path.join(out_dir, f"debug_grid_top_{pt_idx}.jpg"), f_top)
            if f_side is not None:
                cv2.circle(f_side, p_side, 8, (0, 0, 255), -1)
                cv2.drawMarker(f_side, p_side, (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.imwrite(os.path.join(out_dir, f"debug_grid_side_{pt_idx}.jpg"), f_side)
        else:
            print("  [ERROR] Gripper not detected in one or both views.")
            
    # Return to home
    print("\nReturning home...")
    moveG0(-11.87, 239.10, 40.0)
    ser.close()
    
    # Save the calibration dataset
    np.save(os.path.join(out_dir, "grid_calibration_data.npy"), pixel_data)
    print(f"\n[SUCCESS] Calibration complete. {len(pixel_data)}/18 points matched.")
    print(f"Data saved to {out_dir}/grid_calibration_data.npy")

if __name__ == "__main__":
    main()
