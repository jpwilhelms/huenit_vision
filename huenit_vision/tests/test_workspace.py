import pytest
from huenit_vision.workspace import check_reachable
from robot_control.robot import checkXYZ

def test_compare_with_legacy_check():
    # Test a grid of coordinates and verify behavior matches or handles crashes
    for z in range(-50, 150, 10):
        for x in range(-150, 150, 30):
            for y in range(-150, 150, 30):
                # Try running the legacy check
                try:
                    legacy_res = bool(checkXYZ(x, y, z))
                    legacy_crashed = False
                except ValueError:
                    legacy_res = False
                    legacy_crashed = True
                
                res = check_reachable(x, y, z)
                
                if legacy_crashed:
                    # If legacy check crashed, our function should return False
                    assert res is False
                else:
                    assert res == legacy_res

def test_out_of_bound_prevent_value_error():
    # Coordinate that causes ValueError in checkXYZ (math domain error under square root)
    # x=0, y=100, z=300:
    # leng = 100.0 (<= 213.44)
    # z = 300.0 (>= 2.182, > 100.1)
    # domain_val = 3600 - (300 - 148)**2 = 3600 - 23104 = -19504 < 0
    x, y, z = 0.0, 100.0, 300.0
    
    # Confirm checkXYZ raises ValueError
    with pytest.raises(ValueError):
        checkXYZ(x, y, z)
        
    # Confirm check_reachable handles it and returns False
    assert check_reachable(x, y, z) is False

def test_negative_y_rear_limit():
    # If y < 0, and y/leng is close to -1 (so ix = -y/leng > 0.98), it should return False
    # x = 1.0, y = -100.0 => leng = sqrt(1 + 10000) = 100.005
    # ix = 100.0 / 100.005 = 0.99995 > 0.98 => should return False
    assert check_reachable(1.0, -100.0, 10.0) is False
    
    # If y < 0, but y/leng is not close to -1 (e.g. x = 100.0, y = -1.0)
    # leng = sqrt(10000 + 1) = 100.005
    # ix = 1.0 / 100.005 = 0.00999 < 0.98 => should not be limited by rear limit
    # (other limits might still apply, so let's choose coordinates that are otherwise valid)
    # Let's check a point in Branch 3:
    # leng = 150.0, z = 20.0
    # cond1: 150.0 < (16500 + 2000)/17 = 18500/17 = 1088.2 (True)
    # cond2: (150 - 228)**2 + (20 + 4)**2 = (-78)**2 + 24**2 = 6084 + 576 = 6660 < 22500 (True)
    # x = 149.0, y = -10.0 => leng = sqrt(22201 + 100) = 149.33
    # ix = 10 / 149.33 = 0.066 < 0.98 => reachable (True)
    assert check_reachable(149.0, -10.0, 20.0) is True

def test_z_low_boundary():
    # Test for z <= 9.066 in Branch 1 (z between 2.182 and 9.066)
    # At z = 5.0, domain_val = 3025 - 625 = 2400, sqrt(2400) + 61.0 ≈ 109.99
    # If leng <= 109.99, it should return False.
    assert check_reachable(100.0, 0.0, 5.0) is False
    # If leng > 109.99 (and falls through to Branch 3, which is valid), it should return True.
    assert check_reachable(120.0, 0.0, 5.0) is True

def test_branch2_negative_boundaries():
    # Test for Branch 2 (z <= 3.241 and leng <= 224.566)
    # At z = -10.0, domain_val = 20736 - 128^2 = 4352, sqrt(4352) + 81.0 ≈ 146.97
    # If leng <= 146.97, it should return False.
    assert check_reachable(140.0, 0.0, -10.0) is False
    # If leng > 146.97, it should return True.
    assert check_reachable(150.0, 0.0, -10.0) is True
    
    # Test for domain_val < 0.0 in Branch 2:
    # At z = -300.0, (z + 138)^2 = 162^2 = 26244 > 20736, so domain_val < 0
    # It must handle it safely and return False.
    assert check_reachable(200.0, 0.0, -300.0) is False

