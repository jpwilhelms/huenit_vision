import cv2
import numpy as np
import pytest
from huenit_vision.aruco import detect_marker_0

def test_detect_marker_0_none_frame():
    assert detect_marker_0(None) is None

def test_detect_marker_0_invalid_format():
    assert detect_marker_0(123) is None
    assert detect_marker_0(np.array([1])) is None

def test_detect_marker_0_blank_frame():
    blank_frame = np.zeros((200, 200), dtype=np.uint8)
    assert detect_marker_0(blank_frame) is None

def test_detect_marker_0_id_0():
    # Create a white test frame
    test_frame = np.ones((400, 400, 3), dtype=np.uint8) * 255
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Generate marker ID 0
    marker_size = 200
    try:
        # OpenCV 4.7+ API
        marker_img = cv2.aruco.generateImageMarker(dictionary, 0, marker_size)
    except AttributeError:
        # OpenCV 4.6- API
        marker_img = cv2.aruco.drawMarker(dictionary, 0, marker_size)
        
    # Embed marker at center of test_frame
    x_start = 100
    y_start = 100
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 0] = marker_img
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 1] = marker_img
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 2] = marker_img
    
    center = detect_marker_0(test_frame)
    assert center is not None, "Marker ID 0 was not detected"
    # Expected center is around (200, 200) since marker of size 200 is embedded at (100, 100)
    assert abs(center[0] - 200) <= 2
    assert abs(center[1] - 200) <= 2

def test_detect_marker_0_id_1():
    # Create a white test frame
    test_frame = np.ones((400, 400, 3), dtype=np.uint8) * 255
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Generate marker ID 1
    marker_size = 200
    try:
        # OpenCV 4.7+ API
        marker_img = cv2.aruco.generateImageMarker(dictionary, 1, marker_size)
    except AttributeError:
        # OpenCV 4.6- API
        marker_img = cv2.aruco.drawMarker(dictionary, 1, marker_size)
        
    # Embed marker at center of test_frame
    x_start = 100
    y_start = 100
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 0] = marker_img
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 1] = marker_img
    test_frame[y_start:y_start+marker_size, x_start:x_start+marker_size, 2] = marker_img
    
    center = detect_marker_0(test_frame)
    # detect_marker_0 should return None when only ID 1 is in the frame
    assert center is None
