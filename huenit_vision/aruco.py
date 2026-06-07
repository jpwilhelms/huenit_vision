"""
huenit_vision/aruco.py
Module for ArUco marker detection compatible with OpenCV 4.6 and 4.7+.
"""

import cv2
import numpy as np

def detect_marker_0(frame) -> tuple[int, int] | None:
    """
    Detects ArUco marker with ID 0 in the given frame across four dictionaries:
    DICT_4X4_50, DICT_4X4_100, DICT_6X6_50, DICT_APRILTAG_36h11.
    
    Supports both legacy OpenCV 4.6 API and new OpenCV 4.7+ ArucoDetector API.
    
    Args:
        frame: OpenCV image frame (numpy array).
        
    Returns:
        tuple[int, int] | None: (cX, cY) coordinates of the marker center,
                                or None if not detected.
    """
    if not hasattr(cv2, "aruco") or cv2.aruco is None:
        return None

    if frame is None:
        return None
    if not hasattr(frame, "shape") or not hasattr(frame, "size"):
        return None
    if frame.size <= 0:
        return None
    if not hasattr(frame, "dtype") or frame.dtype != np.uint8:
        return None
    if len(frame.shape) not in (2, 3):
        return None
        
    # List of dictionary constants to search in order
    dict_names = [
        "DICT_4X4_50",
        "DICT_4X4_100",
        "DICT_6X6_50",
        "DICT_APRILTAG_36h11"
    ]
    
    for name in dict_names:
        # Get dictionary constant dynamically to avoid errors on extremely old versions
        try:
            dict_id = getattr(cv2.aruco, name, None)
            if dict_id is None:
                # Check uppercase variation as fallback (e.g. DICT_APRILTAG_36H11)
                dict_id = getattr(cv2.aruco, name.upper(), None)
        except (AttributeError, cv2.error):
            continue
            
        if dict_id is None:
            continue
            
        try:
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        except (AttributeError, cv2.error):
            # Fallback for older OpenCV naming if needed
            continue
            
        try:
            # Try new OpenCV 4.7+ ArucoDetector API
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejected = detector.detectMarkers(frame)
        except (AttributeError, cv2.error):
            # Fallback to legacy OpenCV 4.6 and older API
            try:
                parameters = cv2.aruco.DetectorParameters_create()
                corners, ids, rejected = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            except (AttributeError, cv2.error):
                # If even legacy fails due to missing cv2.aruco attributes
                continue
                
        try:
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == 0:
                        pts = corners[i][0]
                        # Calculate center coordinates (mean of X and Y of the 4 corners)
                        cX = int(np.mean(pts[:, 0]))
                        cY = int(np.mean(pts[:, 1]))
                        return cX, cY
        except (AttributeError, cv2.error, IndexError, KeyError, TypeError):
            continue
                    
    return None
