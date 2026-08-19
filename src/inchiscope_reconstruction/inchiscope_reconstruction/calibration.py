"""Loads the two fixed calibration inputs reconstruction needs.

Deliberately loads camera intrinsics from wherever inchiscope_bringup's
canonical camera_info.yaml lives (passed in as a path -- see
inchiscope_reconstruction/README.md) rather than keeping a second copy in
this package's own config. inchiscope_camera already hit and fixed a bug
from two camera_info.yaml files drifting out of sync; duplicating it again
here would just reintroduce the same failure mode.
"""

import numpy as np
import yaml


def load_camera_intrinsics(path):
    with open(path) as f:
        info = yaml.safe_load(f)
    K = np.array(info['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(info['distortion_coefficients']['data'], dtype=np.float64)
    distortion_model = info.get('distortion_model', 'plumb_bob')
    return K, D, distortion_model, info['image_width'], info['image_height']


def quat_to_rotmat(x, y, z, w):
    """Same formula as inchiscope_camera/scripts/calibrate_hand_eye.py's
    helper of the same name (validated there against 2000 random rotations
    to machine precision) -- duplicated rather than imported since
    scripts/ isn't an installed, importable module and this is a separate
    ROS package."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def load_hand_eye_transform(path):
    """Returns (R, t) for the aurora_sensor_0 -> naneye_camera transform,
    i.e. the camera's pose expressed in the Aurora sensor frame -- the
    same quantity calibrate_hand_eye.py solves for and the same semantics
    as the static_transform_publisher arguments it prints."""
    with open(path) as f:
        data = yaml.safe_load(f)
    t = np.array([data['translation']['x'], data['translation']['y'], data['translation']['z']])
    q = data['rotation_xyzw']
    R = quat_to_rotmat(q['x'], q['y'], q['z'], q['w'])
    return R, t
