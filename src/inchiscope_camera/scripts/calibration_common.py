"""Corner-detection helper shared by calibrate_camera.py (standard/pinhole
lenses) and calibrate_camera_fisheye.py (>=~90-100deg lenses) -- kept in one
place so the two scripts can't drift apart on how corners are actually
found; detection itself doesn't depend on which distortion model the
intrinsics will eventually be fit with.
"""

import cv2


def find_corners(gray, corners_x, corners_y):
    """Tries the robust SB detector first (handles the uneven ring-light
    illumination typical of an endoscope's onboard LED better than the
    classic detector), falls back to the classic one."""
    size = (corners_x, corners_y)
    found, corners = cv2.findChessboardCornersSB(gray, size)
    if found:
        return found, corners
    found, corners = cv2.findChessboardCorners(
        gray, size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
    )
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return found, corners
