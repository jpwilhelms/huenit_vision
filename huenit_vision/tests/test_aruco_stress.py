import cv2
import numpy as np
import pytest
from huenit_vision.aruco import detect_marker_0

def generate_synthetic_marker(dict_id, marker_id, marker_size):
    """
    Helper to generate a binary ArUco marker image compatible with OpenCV 4.6 and 4.7+.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    try:
        return cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
    except AttributeError:
        return cv2.aruco.drawMarker(dictionary, marker_id, marker_size)

def create_test_frame(dict_id=cv2.aruco.DICT_4X4_50, marker_id=0, marker_size=100, canvas_size=(400, 400), center_pos=(200, 200)):
    """
    Embeds an ArUco marker at a specific center position in a white canvas.
    """
    canvas = np.ones((canvas_size[1], canvas_size[0], 3), dtype=np.uint8) * 255
    marker_img = generate_synthetic_marker(dict_id, marker_id, marker_size)
    
    x_start = center_pos[0] - marker_size // 2
    y_start = center_pos[1] - marker_size // 2
    
    # Embed marker
    canvas[y_start:y_start+marker_size, x_start:x_start+marker_size, 0] = marker_img
    canvas[y_start:y_start+marker_size, x_start:x_start+marker_size, 1] = marker_img
    canvas[y_start:y_start+marker_size, x_start:x_start+marker_size, 2] = marker_img
    return canvas

def add_gaussian_noise(image, mean=0, sigma=15):
    noise = np.random.normal(mean, sigma, image.shape).astype(np.int16)
    return np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)

def adjust_brightness(image, factor):
    return np.clip(image.astype(np.float32) * factor, 0, 255).astype(np.uint8)

def rotate_image(image, angle, center_pt):
    h, w = image.shape[:2]
    M = cv2.getRotationMatrix2D(center_pt, angle, 1.0)
    return cv2.warpAffine(image, M, (w, h), borderValue=(255, 255, 255))

def apply_blur(image, kernel_size):
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

# --- Test Cases ---

def test_noise_robustness():
    """
    Tests detection convergence under various levels of Gaussian noise.
    """
    center_pos = (200, 200)
    frame = create_test_frame(marker_size=100, center_pos=center_pos)
    
    # Test levels of sigma
    for sigma in [5, 10, 20, 30]:
        noisy_frame = add_gaussian_noise(frame, sigma=sigma)
        res = detect_marker_0(noisy_frame)
        if res is not None:
            # If detected, verify it converges to the correct center
            dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
            assert dist <= 3, f"Noise sigma={sigma} led to incorrect center {res} (expected {center_pos})"
        else:
            # High noise can cause detection failure (returns None), which is acceptable,
            # but for low noise (e.g. sigma <= 10), it should definitely succeed.
            if sigma <= 10:
                pytest.fail(f"Failed to detect marker under low noise (sigma={sigma})")

def test_brightness_robustness():
    """
    Tests detection convergence under varying brightness levels (dark to bright).
    """
    center_pos = (200, 200)
    frame = create_test_frame(marker_size=100, center_pos=center_pos)
    
    # Test brightness scaling factors
    for factor in [0.3, 0.5, 0.8, 1.2, 1.5, 2.0]:
        bright_frame = adjust_brightness(frame, factor)
        res = detect_marker_0(bright_frame)
        assert res is not None, f"Failed to detect marker with brightness factor={factor}"
        dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
        assert dist <= 3, f"Brightness factor={factor} led to incorrect center {res} (expected {center_pos})"

def test_rotation_robustness():
    """
    Tests detection convergence under slight rotation angles (-15 to 15 degrees).
    """
    center_pos = (200, 200)
    frame = create_test_frame(marker_size=120, center_pos=center_pos)
    
    for angle in [-15, -10, -5, 5, 10, 15]:
        rotated_frame = rotate_image(frame, angle, center_pos)
        res = detect_marker_0(rotated_frame)
        assert res is not None, f"Failed to detect marker with rotation angle={angle}"
        dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
        assert dist <= 3, f"Rotation angle={angle} led to incorrect center {res} (expected {center_pos})"

def test_blur_robustness():
    """
    Tests detection convergence under blur of varying kernel sizes.
    """
    center_pos = (200, 200)
    frame = create_test_frame(marker_size=100, center_pos=center_pos)
    
    for ksize in [3, 5, 7]:
        blurred_frame = apply_blur(frame, ksize)
        res = detect_marker_0(blurred_frame)
        assert res is not None, f"Failed to detect marker with blur kernel size={ksize}"
        dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
        assert dist <= 3, f"Blur kernel size={ksize} led to incorrect center {res} (expected {center_pos})"

def test_scale_robustness():
    """
    Tests detection convergence with different marker scales/sizes.
    """
    center_pos = (200, 200)
    
    # Try different marker sizes
    for size in [40, 80, 150, 250]:
        frame = create_test_frame(marker_size=size, center_pos=center_pos)
        res = detect_marker_0(frame)
        assert res is not None, f"Failed to detect marker of size {size}"
        dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
        assert dist <= 3, f"Marker size={size} led to incorrect center {res} (expected {center_pos})"

def test_combined_stress():
    """
    Tests detection under a combination of noise, brightness, rotation, and blur.
    """
    center_pos = (200, 200)
    frame = create_test_frame(marker_size=120, center_pos=center_pos)
    
    # Apply multiple perturbations
    stressed_frame = rotate_image(frame, 10, center_pos)
    stressed_frame = adjust_brightness(stressed_frame, 0.7)
    stressed_frame = apply_blur(stressed_frame, 3)
    stressed_frame = add_gaussian_noise(stressed_frame, sigma=8)
    
    res = detect_marker_0(stressed_frame)
    assert res is not None, "Failed to detect marker under combined stress conditions"
    dist = np.sqrt((res[0] - center_pos[0])**2 + (res[1] - center_pos[1])**2)
    assert dist <= 4, f"Combined stress led to incorrect center {res} (expected {center_pos})"

@pytest.mark.parametrize("corrupt_input,name", [
    (np.random.randint(0, 256, (300, 300, 3), dtype=np.uint8), "random_noise"),
    (np.zeros((300, 300, 3), dtype=np.uint8), "all_black"),
    (np.ones((300, 300, 3), dtype=np.uint8) * 255, "all_white"),
    (np.zeros((2, 2, 3), dtype=np.uint8), "small_frame_2x2"),
    (np.zeros((0, 0, 3), dtype=np.uint8), "empty_frame_0x0"),
    (np.zeros((100, 0, 3), dtype=np.uint8), "empty_frame_100x0"),
    (np.zeros((100,), dtype=np.uint8), "1d_array"),
    (np.zeros((100, 100, 3, 1), dtype=np.uint8), "4d_array"),
    (np.zeros((100, 100, 3), dtype=np.float32), "float32_array"),
    (None, "None_input"),
    (12345, "int_input"),
    ("string_image", "str_input"),
    ([1, 2, 3], "list_input"),
])
def test_crash_prevention(corrupt_input, name):
    """
    Verifies that the detection function handles invalid, corrupt, or noisy inputs without crashing.
    """
    try:
        res = detect_marker_0(corrupt_input)
        assert res is None, f"Expected None for {name}, but got {res}"
    except cv2.error as e:
        pytest.fail(f"cv2.error raised on {name}: {e}")
    except Exception as e:
        pytest.fail(f"Unexpected exception {type(e).__name__} raised on {name}: {e}")


def test_invalid_markers_and_other_ids():
    """
    Verifies that only marker ID 0 is detected and markers with other IDs return None.
    """
    # Generate marker ID 1
    frame_id1 = create_test_frame(marker_id=1, marker_size=100)
    assert detect_marker_0(frame_id1) is None
    
    # Generate marker ID 5
    frame_id5 = create_test_frame(marker_id=5, marker_size=100)
    assert detect_marker_0(frame_id5) is None
    
    # Generate marker ID 10
    frame_id10 = create_test_frame(marker_id=10, marker_size=100)
    assert detect_marker_0(frame_id10) is None
