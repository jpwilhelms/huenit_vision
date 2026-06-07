#!/usr/bin/env python3
import sys
import os
import time
import cv2
import numpy as np

# Import local robot_control library
from robot_control.robot import openSerial, moveG0, checkXYZ, goHome

def capture_frame(dev_id=2):
    cap = cv2.VideoCapture(dev_id)
    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {dev_id}")
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(15):
        cap.read()
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None

def get_motion_centroid(f1, f2, label, output_dir):
    if f1 is None or f2 is None:
        print(f"[{label}] Error: Capture failed")
        return None
        
    gray1 = cv2.cvtColor(f1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(f2, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray1, gray2)
    
    # Try different thresholds to find the motion blob of the tool head
    for th in [10, 15, 20, 25, 30]:
        _, thresh = cv2.threshold(diff, th, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        for c in contours:
            area = cv2.contourArea(c)
            if area > 100:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    # Spatial filter: tool head should be in cY > 100 to avoid base/shoulder column motion
                    if cY > 100:
                        candidates.append((cX, cY, area))
                        
        if candidates:
            candidates.sort(key=lambda x: x[2], reverse=True)
            cX, cY, area = candidates[0]
            print(f"[{label}] Threshold {th}: Success! Centroid: ({cX}, {cY}), Area: {area:.1f}")
            # Save visual diff
            cv2.circle(diff, (cX, cY), 8, 255, -1)
            cv2.imwrite(os.path.join(output_dir, f"motion_diff_{label}.jpg"), diff)
            return cX, cY
            
    print(f"[{label}] Error: No motion blob detected at any threshold")
    return None

def main():
    output_dir = "./calib_results"
    os.makedirs(output_dir, exist_ok=True)
    
    print("Opening serial connection...")
    try:
        ser = openSerial()
        ser.timeout = 2.0
        ser.rts = False
    except Exception as e:
        print(f"Serial open failed: {e}")
        return
        
    print("Homing robot before calibration to clear step losses...")
    ser.write(b"M17\nG90\nM1008 A5\n")
    time.sleep(8.0)
    
    pts = [
        ("P1", 0.0, 200.0),
        ("P2", 100.0, 240.0),
        ("P3", -100.0, 240.0)
    ]
    
    pixel_pts = []
    robot_pts = []
    z_height = -30.0 # Height for the calibration sweep
    
    for label, rx, ry in pts:
        print(f"\n--- Recalibrating Point {label}: X={rx}, Y={ry} ---")
        
        # Move to start position
        if not checkXYZ(rx, ry, z_height) or not checkXYZ(rx, ry + 25.0, z_height):
            print(f"[WARN] Target unreachable at Z={z_height}")
            continue
            
        moveG0(rx, ry, z_height)
        time.sleep(3.5)
        f1 = capture_frame(2) # Top camera is dev 2
        
        # Translate in Y
        moveG0(rx, ry + 25.0, z_height)
        time.sleep(3.5)
        f2 = capture_frame(2)
        
        centroid = get_motion_centroid(f1, f2, label, output_dir)
        if centroid is not None:
            pixel_pts.append(centroid)
            robot_pts.append((rx, ry + 12.5)) # Midpoint of Y motion
            
    # Return Home
    print("\nReturning home...")
    moveG0(-11.87, 239.10, 40.0)
    ser.close()
    
    if len(pixel_pts) == 3:
        src = np.array(pixel_pts, dtype=np.float32)
        dst = np.array(robot_pts, dtype=np.float32)
        M = cv2.getAffineTransform(src, dst)
        print("\n==============================")
        print("SOLVED AFFINE MATRIX (Pixel -> Robot):")
        print(M.tolist())
        print("==============================")
        
        np.save(os.path.join(output_dir, "M_affine_overhead.npy"), M)
        print(f"Matrix saved to {output_dir}/M_affine_overhead.npy")
    else:
        print("\nCalibration failed: not enough points detected")

if __name__ == "__main__":
    main()
