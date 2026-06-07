# paul update 23.10.18
# paul udpate 24.01.04 (bug fix)

import math
import serial
from serial.tools import list_ports
from typing import Tuple
import time
import sys

def extract_floats(text):
    floats = []
    current_number = ""
    for char in text:
        if char.isdigit() or char in ".+-":
            current_number += char
        else:
            if current_number:
                try:
                    floats.append(float(current_number))
                except ValueError:
                    pass
                current_number = ""
    if current_number:
        try:
            floats.append(float(current_number))
        except ValueError:
            pass
    return floats

def scan_ports():
    ports = list(serial.tools.list_ports.comports())
    available_ports = [(port.device, port.description, port.vid, port.pid, port.manufacturer, port.serial_number) for
                       port in ports]
    return available_ports

def get_my_device():
    port_list = scan_ports()
    for item in port_list:
        if 'FTDI' in item:
            return (item[0], item[1])
    # Fallback to first available port if no explicit FTDI string
    if port_list:
        return (port_list[0][0], port_list[0][1])
    return None

def openSerial(baudrate=115200, timeout=1000):
    dev = get_my_device()
    if dev is None:
        raise RuntimeError("No serial device found.")
    return serial.Serial(dev[0], baudrate, timeout=timeout)

def checkConnection():
    command = "M400\n"
    try:
        ser = openSerial()
        ser.write(str.encode(command))
        strt = time.time()
        while True:
            if (time.time() - strt > 0.1):
                ser.close()
                return False
            read_data = ser.readline().decode("utf-8")
            if read_data:
                if read_data.find("ok") > -1:
                    break
        ser.close()
        return True
    except Exception:
        return False

def sendCommand(command: str) -> Tuple[bool, str, str]:
    ser = openSerial()
    ser.rts = False
    is_success = True
    error_msg = ""
    additional_info = ""
    ser.write(str.encode(command))
    while True:
        try:
            inputLine = ser.readline().decode("utf-8")
            if len(inputLine) > 0:
                if inputLine.find("ok") > -1:
                    break
                elif inputLine.find("Unknown") > -1:
                    error_msg = "unknown command received"
                    is_success = False
                elif inputLine.find("Current Point") > -1:
                    additional_info = inputLine
                elif inputLine.find("Module Type") > -1:
                    additional_info = inputLine
                else:
                    additional_info = inputLine
            else:
                is_success = False
                error_msg = "No lines to read"
                break
        except Exception as e:
            is_success = False
            error_msg = str(e)
            break
        time.sleep(0.00001)
    ser.close()
    return is_success, error_msg, additional_info

def goHome():
    command = "M1008 A5\n"
    sendCommand(command)

def sendCommandNoReturn(command: str):
    ser = openSerial()
    ser.write(str.encode(command))

def checkXYZ(x, y, z):
    leng = math.sqrt(x * x + y * y)
    if (leng <= 213.44 and z >= 2.182):
        if (z <= 9.066):
            if (not (leng > 61 + math.pow((55 * 55 - (z - 30) ** 2), (0.5)))):
                return 0
        elif (z <= 100.1):
            if (not (leng > math.pow((152 ** 2 - (z - 147) ** 2), (0.5)) + 48)):
                return 0
        else:
            if (not (leng > math.pow((3600 - (z - 148) ** 2), (0.5)) + 153.5)):
                return 0
    elif (leng <= 224.566 and z <= 3.241):
        if (not (leng > math.pow((144 ** 2 - (z + 138) ** 2), (0.5)) + 81)):
            return 0
    else:
        if (not ((leng < (16500 + 100 * z) / 17) and ((leng - 228) ** 2 + (z + 4) ** 2 < 150 ** 2))):
            return 0
    if (y < 0):
        ix = (-1 * y) / leng
        if 0.98 < ix:
            return 0
    return 1

def moveG0(*args):
    if len(args) == 1 and isinstance(args[0], (tuple, list)):
        x, y, z = args[0]
    elif len(args) == 3:
        x, y, z = args
    else:
        raise ValueError("moveG0 expects either three separate values or a single tuple/list (x, y, z).")
    command = f"G0 X{x} Y{y} Z{z}\n"
    sendCommand(command)
    sendCommand("M400\n")

def moveG1(*args):
    if len(args) == 1 and isinstance(args[0], (tuple, list)):
        x, y, z = args[0]
    elif len(args) == 3:
        x, y, z = args
    else:
        raise ValueError("moveG1 expects either three separate values or a single tuple/list (x, y, z).")
    command = f"G1 X{x} Y{y} Z{z}\n"
    sendCommand(command)
    sendCommand("M400\n")

def pumpOn():
    command = "M1401 A0\n"
    sendCommand(command)
    command = "M1400 A1023\n"
    sendCommand(command)

def pumpOff():
    command = "M1400 A0\n"
    sendCommand(command)

def pump(power):
    command = "M1400 A" + str(power) + "\n"
    sendCommand(command)

def valveOn():
    command = "M1401 A1\n"
    sendCommand(command)

def valveOff():
    command = "M1401 A0\n"
    sendCommand(command)

def suctionOn():
    valveOff()
    pumpOn()

def suctionOff():
    pumpOff()
    valveOn()
    time.sleep(0.3)
    valveOff()

def gripper(state):
    if state == 0:
        pumpOff()
        valveOn()
        time.sleep(0.3)
        valveOff()
    elif state == 1:
        valveOff()
        pump(623)
    elif state == 2:
        valveOn()
        pump(623)

def set_current_position():
    sendCommand("M1500 B4\n")

def moveAngle(a, b, c):
    command = "M1005 A" + str(a) + " B" + str(b) + " C" + str(c) + "\n"
    sendCommand(command)
    sendCommand("M400\n")

def moveAngle_noM400(a, b, c):
    command = "M1005 A" + str(a) + " B" + str(b) + " C" + str(c) + "\n"
    sendCommand(command)

def moveZ0_M400(z):
    command = "G0 Z" + str(z) + "\n"
    sendCommand(command)
    sendCommand("M400\n")

def moveZ0(z):
    command = "G0 Z" + str(z) + "\n"
    sendCommand(command)

def freeMod():
    sendCommand("M84\n")

def unsetFreeMod():
    sendCommand("M17\n")

def homingAxis():
    sendCommand("M1008 A1\n")
    sendCommand("G90\n")

def getLoc():
    info1, info2, info3 = sendCommand("M1008 A3\n")
    withFlt = extract_floats(info3)
    return withFlt

def getDeg():
    info1, info2, info3 = sendCommand("M1008 A2\n")
    withFlt = extract_floats(info3)
    return withFlt

def suctionAngle(angle, speed=60):
    normalized_angle = angle % 360
    if normalized_angle < 0:
        normalized_angle += 360
    if (normalized_angle >= 270):
        print("Unsupported degree")
        return
    if (speed <= 0):
        print("Unsupported speed")
        return
    sendCommand("M9996 A1B" + str(normalized_angle) + "C" + str(speed) + "\n")
