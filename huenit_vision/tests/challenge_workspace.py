import sys
import math
import random
import itertools
from huenit_vision.workspace import check_reachable
from robot_control.robot import checkXYZ

def generate_points():
    # 1. Grid-based points (total: 101 * 101 * 51 = 520,251)
    xs = [x * 6.0 for x in range(-50, 51)]
    ys = [y * 6.0 for y in range(-50, 51)]
    zs = [z * 8.0 for z in range(-25, 26)]
    
    for x, y, z in itertools.product(xs, ys, zs):
        yield x, y, z

    # 2. Boundary and Edge points (around the constants used in conditions)
    lengs = [213.439, 213.44, 213.441, 224.565, 224.566, 224.567, 0.0, 1.0]
    zs_edge = [2.181, 2.182, 2.183, 9.065, 9.066, 9.067, 100.09, 100.1, 100.11,
               3.240, 3.241, 3.242, 147.0, 148.0, -138.0, -4.0, 0.0]
    for l in lengs:
        for z in zs_edge:
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                rad = math.radians(angle)
                x = l * math.cos(rad)
                y = l * math.sin(rad)
                yield x, y, z

    # 3. Extremes and Special Floats
    extremes = [1e10, 1e20, 1e150, 1e300, -1e10, -1e20, -1e150, -1e300]
    underflows = [1e-10, 1e-150, 1e-300, -1e-10, -1e-150, -1e-300]
    special_floats = [float('nan'), float('inf'), float('-inf')]
    
    all_specials = extremes + underflows + special_floats
    for val in all_specials:
        yield val, 0.0, 0.0
        yield 0.0, val, 0.0
        yield 0.0, 0.0, val
        yield val, val, val
        yield val, -val, val

    # 4. Randomized points in various scales
    random.seed(42)
    for _ in range(50000):
        x = random.uniform(-500.0, 500.0)
        y = random.uniform(-500.0, 500.0)
        z = random.uniform(-300.0, 300.0)
        yield x, y, z
        
    for _ in range(10000):
        x = random.uniform(-1e10, 1e10)
        y = random.uniform(-1e10, 1e10)
        z = random.uniform(-1e10, 1e10)
        yield x, y, z

def run_stress_test():
    total_count = 0
    
    checkXYZ_success = 0
    checkXYZ_exceptions = {}
    
    check_reachable_success = 0
    check_reachable_exceptions = {}
    
    crashes = []
    correctness_mismatches = []
    
    print("Starting stress test generation and validation...", flush=True)
    
    for x, y, z in generate_points():
        total_count += 1
        
        # 1. Run checkXYZ
        xyz_val = None
        xyz_exception = None
        try:
            xyz_val = checkXYZ(x, y, z)
            checkXYZ_success += 1
        except Exception as e:
            exc_name = type(e).__name__
            checkXYZ_exceptions[exc_name] = checkXYZ_exceptions.get(exc_name, 0) + 1
            xyz_exception = e
            
        # 2. Run check_reachable
        reach_val = None
        reach_exception = None
        try:
            reach_val = check_reachable(x, y, z)
            check_reachable_success += 1
        except Exception as e:
            exc_name = type(e).__name__
            check_reachable_exceptions[exc_name] = check_reachable_exceptions.get(exc_name, 0) + 1
            reach_exception = e

        # 3. Check for exceptions in check_reachable
        if reach_exception is not None:
            crashes.append({
                "coords": (x, y, z),
                "exception": type(reach_exception).__name__,
                "message": str(reach_exception),
                "checkXYZ_crashed": xyz_exception is not None
            })
        elif xyz_exception is not None:
            # checkXYZ crashed, check_reachable should return False (valid behavior)
            if reach_val is not False:
                correctness_mismatches.append({
                    "coords": (x, y, z),
                    "type": "checkXYZ_crashed_but_reachable_true",
                    "details": f"checkXYZ raised {type(xyz_exception).__name__}: {xyz_exception}, but check_reachable returned True"
                })
        else:
            # Both executed successfully, compare results
            legacy_bool = bool(xyz_val)
            if reach_val != legacy_bool:
                correctness_mismatches.append({
                    "coords": (x, y, z),
                    "type": "mismatch",
                    "details": f"checkXYZ={xyz_val} (bool: {legacy_bool}) vs check_reachable={reach_val}"
                })

        if total_count % 100000 == 0:
            print(f"Processed {total_count} points...", flush=True)

    print("\n--- STRESS TEST RESULT SUMMARY ---")
    print(f"Total points tested: {total_count}")
    print(f"checkXYZ successfully returned: {checkXYZ_success}")
    print(f"checkXYZ exceptions: {checkXYZ_exceptions}")
    print(f"check_reachable successfully returned: {check_reachable_success}")
    print(f"check_reachable exceptions: {check_reachable_exceptions}")
    
    print(f"\n--- check_reachable CRASHES ({len(crashes)}) ---")
    for idx, c in enumerate(crashes[:10]):
        print(f"Crash {idx+1}: Coords={c['coords']} Exc={c['exception']} Msg={c['message']} (checkXYZ also crashed: {c['checkXYZ_crashed']})")
    if len(crashes) > 10:
        print(f"... and {len(crashes) - 10} more crashes.")

    print(f"\n--- CORRECTNESS MISMATCHES ({len(correctness_mismatches)}) ---")
    for idx, m in enumerate(correctness_mismatches[:10]):
        print(f"Mismatch {idx+1}: Coords={m['coords']} Type={m['type']} Details={m['details']}")
    if len(correctness_mismatches) > 10:
        print(f"... and {len(correctness_mismatches) - 10} more mismatches.")

    # Return status: we consider it successful if we collected the stats successfully.
    # The findings are documented and will be presented in the handoff.
    sys.exit(0)

if __name__ == "__main__":
    run_stress_test()
