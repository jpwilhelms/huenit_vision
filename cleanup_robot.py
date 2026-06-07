import sys
import time
import os

sys.path.append("/home/jpw/ai-projects/robot-arm-video-feedback")
from robot_control.robot import openSerial

print("Opening serial connection for safety cleanup...")
try:
    ser = openSerial()
    ser.timeout = 2.0; ser.rts = False
except Exception as e:
    print(f"Failed to open serial: {e}")
    sys.exit(1)

print("Turning suction pump OFF...")
ser.write(b"M1400 A0\nM1401 A1\n")
time.sleep(1.0)
ser.write(b"M1401 A0\n")

print("Lifting arm to Z=40...")
ser.write(b"G1 Z40.0 F2000\nM400\n")
time.sleep(2.0)

print("Parking arm at (-150, 150)...")
ser.write(b"G1 X-150.0 Y150.0 Z40.0 F3000\nM400\n")
time.sleep(3.0)

ser.close()
print("Cleanup complete. Robot is parked and idle.")
