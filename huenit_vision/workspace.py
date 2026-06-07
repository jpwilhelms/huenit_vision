"""
Workspace kinematics module for Huenit robot arm.

This module provides the check_reachable function to verify if a given 3D coordinate (x, y, z)
is within the physical workspace limits of the Huenit robot arm.
"""

import math

def check_reachable(x: float, y: float, z: float) -> bool:
    """
    Check if a 3D coordinate (x, y, z) is within the robot's reachable workspace.

    This function reimplements the checkXYZ kinematic boundary checks from robot_control.robot,
    converting its outputs to native boolean values (True for reachable/1, False for unreachable/0).
    It includes additional safety checks to prevent ValueError (math domain error) on out-of-bounds input values.

    Args:
        x (float): The X-coordinate in mm.
        y (float): The Y-coordinate in mm.
        z (float): The Z-coordinate in mm.

    Returns:
        bool: True if the point is reachable, False otherwise.
    """
    # Preliminary bounding-box check to prevent overflows with extreme coordinates
    if abs(x) > 1000.0 or abs(y) > 1000.0 or abs(z) > 1000.0:
        return False

    # Compute the radial distance in the XY-plane using hypot to prevent overflow
    leng = math.hypot(x, y)

    # Branch 1: Close-range high workspace boundary
    if leng <= 213.44 and z >= 2.182:
        if z <= 9.066:
            domain_val = 3025.0 - (z - 30.0) ** 2
            if domain_val < 0.0:
                return False
            if not (leng > 61.0 + math.sqrt(domain_val)):
                return False
        elif z <= 100.1:
            domain_val = 23104.0 - (z - 147.0) ** 2
            if domain_val < 0.0:
                return False
            if not (leng > math.sqrt(domain_val) + 48.0):
                return False
        else:
            domain_val = 3600.0 - (z - 148.0) ** 2
            if domain_val < 0.0:
                return False
            if not (leng > math.sqrt(domain_val) + 153.5):
                return False

    # Branch 2: Outer-range low workspace boundary
    elif leng <= 224.566 and z <= 3.241:
        domain_val = 20736.0 - (z + 138.0) ** 2
        if domain_val < 0.0:
            return False
        if not (leng > math.sqrt(domain_val) + 81.0):
            return False

    # Branch 3: Default/outermost workspace boundary
    else:
        # Check boundary conditions defined by the linear and circular equations:
        # 1. Linear slope bound: leng < (16500 + 100 * z) / 17
        # 2. Torus/Sphere reach bound: (leng - 228)**2 + (z + 4)**2 < 150**2
        if not (leng < (16500.0 + 100.0 * z) / 17.0):
            return False
        if not ((leng - 228.0) ** 2 + (z + 4.0) ** 2 < 22500.0):
            return False

    # Rear rotation limit (prevents collision/twist in the negative Y direction)
    if y < 0.0:
        # Note: If y < 0, then leng > 0 is mathematically guaranteed.
        ix = (-1.0 * y) / leng
        if ix > 0.98:
            return False

    return True
