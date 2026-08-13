#!/usr/bin/env python3
"""Interactive intrinsic calibration for the NanEye/OV6948 capture-card camera.

Two subcommands:
    capture     Live view against /dev/videoN; press SPACE to grab a frame
                when the checkerboard is detected and highlighted, 'q' to
                finish. Saves accepted frames as PNGs into --out-dir.
    calibrate   Runs cv2.calibrateCamera over the saved frames, reports
                reprojection error, and writes camera_matrix/distortion to a
                YAML file in the same layout ROS's camera_info expects
                (see inchiscope_camera/README.md and camera_node.py's
                /camera/camera_info TODO).

Usage:
    python3 calibrate_camera.py capture --device /dev/video0 --square-size-mm 3.0 --out-dir calib_frames/
    python3 calibrate_camera.py calibrate --frames-dir calib_frames/ --square-size-mm 3.0 --out camera_info.yaml

--square-size-mm must match whichever printed board size you actually used
(see generate_calibration_target.py) -- get this wrong and every downstream
triangulation/depth number silently comes out at the wrong physical scale.

Board size defaults to 9x6 internal corners (10x7 squares), matching
generate_calibration_target.py -- override with --corners-x/--corners-y if
you print a different-sized board.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml


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


def cmd_capture(args):
    os.makedirs(args.out_dir, exist_ok=True)
    cap = cv2.VideoCapture(args.device)
    if args.pixel_format:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.pixel_format))
    if args.width and args.height:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f'Failed to open {args.device}', file=sys.stderr)
        sys.exit(1)

    saved = 0
    print("SPACE = capture when the board is detected (drawn in colour), "
          "q = finish. Move/tilt the board between captures to cover the "
          "whole frame and a range of angles -- 15-25 good captures is "
          "typical.")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, args.corners_x, args.corners_y)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (args.corners_x, args.corners_y), corners, found)
        cv2.putText(display, f'saved: {saved}', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.imshow('calibrate_camera - capture', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and found:
            path = os.path.join(args.out_dir, f'frame_{saved:03d}.png')
            cv2.imwrite(path, frame)
            saved += 1
            print(f'saved {path}')

    cap.release()
    cv2.destroyAllWindows()
    print(f'Captured {saved} frames to {args.out_dir}')
    if saved < 10:
        print('Fewer than 10 captures -- calibration quality will likely be '
              'poor, consider capturing more before running `calibrate`.')


def cmd_calibrate(args):
    paths = sorted(glob.glob(os.path.join(args.frames_dir, '*.png')))
    if not paths:
        print(f'No .png frames found in {args.frames_dir}', file=sys.stderr)
        sys.exit(1)

    objp = np.zeros((args.corners_x * args.corners_y, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.corners_x, 0:args.corners_y].T.reshape(-1, 2)
    objp *= args.square_size_mm

    objpoints = []
    imgpoints = []
    image_size = None
    used, skipped = 0, 0

    for path in paths:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        found, corners = find_corners(gray, args.corners_x, args.corners_y)
        if not found:
            print(f'  no board found in {path}, skipping')
            skipped += 1
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        used += 1

    if used < 10:
        print(f'Only {used} usable frames (skipped {skipped}) -- capture more '
              'before trusting this calibration.', file=sys.stderr)

    reproj_err, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None,
    )

    print(f'Used {used}/{len(paths)} frames, skipped {skipped}')
    print(f'RMS reprojection error: {reproj_err:.4f} px')
    if reproj_err > 1.0:
        print('  >1px is high for this image size -- check the board is flat, '
              'square-size-mm is correct, and frames cover a range of angles/'
              'positions rather than all being near-identical.')
    print('Camera matrix (K):')
    print(camera_matrix)
    print('Distortion coefficients (k1 k2 p1 p2 k3):')
    print(dist_coeffs.ravel())

    # ROS sensor_msgs/CameraInfo-compatible layout, so this can be loaded by
    # camera_info_manager or copied into inchiscope_camera's params once
    # /camera/camera_info publishing is wired up (see camera_node.py's TODO).
    camera_info = {
        'image_width': image_size[0],
        'image_height': image_size[1],
        'camera_name': 'naneye',
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': camera_matrix.flatten().tolist(),
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1, 'cols': 5,
            'data': dist_coeffs.flatten().tolist(),
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3,
            'data': np.eye(3).flatten().tolist(),
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist(),
        },
    }
    with open(args.out, 'w') as f:
        yaml.safe_dump(camera_info, f, default_flow_style=None, sort_keys=False)
    print(f'Wrote {args.out}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--corners-x', type=int, default=9, help='internal corners, horizontal (default: 9, matches generate_calibration_target.py)')
    common.add_argument('--corners-y', type=int, default=6, help='internal corners, vertical (default: 6)')
    common.add_argument('--square-size-mm', type=float, required=True, help='real-world size of one printed square, in mm -- must match the board you actually used')

    p_capture = sub.add_parser('capture', parents=[common])
    p_capture.add_argument('--device', default='/dev/video0')
    p_capture.add_argument('--pixel-format', default='MJPG')
    p_capture.add_argument('--width', type=int, default=0)
    p_capture.add_argument('--height', type=int, default=0)
    p_capture.add_argument('--out-dir', default='calib_frames')
    p_capture.set_defaults(func=cmd_capture)

    p_calibrate = sub.add_parser('calibrate', parents=[common])
    p_calibrate.add_argument('--frames-dir', default='calib_frames')
    p_calibrate.add_argument('--out', default='camera_info.yaml')
    p_calibrate.set_defaults(func=cmd_calibrate)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
