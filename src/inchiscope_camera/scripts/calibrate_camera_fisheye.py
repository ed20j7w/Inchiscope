#!/usr/bin/env python3
"""Intrinsic calibration for the NanEye/OV6948 capture-card camera, for
lenses at or above roughly 90-100deg field of view.

Uses OpenCV's fisheye/equidistant model (cv2.fisheye.calibrate, 4
distortion coefficients k1-k4) instead of calibrate_camera.py's standard
pinhole + radial-tangential model (cv2.calibrateCamera, 5 coefficients).
The NanEye ships in variants from 90deg up to 160deg field of view -- a
120deg unit is a real, named ams-OSRAM part (NEC_B&W_SGA_FOV120_F4.0) -- and
the plain pinhole model is only a good approximation up to roughly
90-100deg; past that its residual error grows fastest exactly at the frame
edges, which no amount of recalibrating *with that same model* can fix,
since it just refits the same wrong shape again.

Same two subcommands as calibrate_camera.py, and `capture` is in fact the
exact same code (imported, not copied) -- capturing frames doesn't depend
on which distortion model you'll fit them with afterward:
    capture     Live view against /dev/videoN; press SPACE to grab a frame
                when the checkerboard is detected and highlighted, 'q' to
                finish. Saves accepted frames as PNGs into --out-dir.
    calibrate   Runs cv2.fisheye.calibrate over the saved frames, reports
                reprojection error, and writes camera_matrix/distortion to
                a YAML file in the same camera_info layout
                calibrate_camera.py writes (distortion_model: equidistant,
                4 coefficients instead of 5) -- every downstream consumer
                (camera_node, calibrate_hand_eye.py, inchiscope_reconstruction)
                branches on that field automatically.

Usage:
    python3 calibrate_camera_fisheye.py capture --device /dev/video0 --width 1280 --height 720 \
        --crop-x 391 --crop-y 111 --crop-width 480 --crop-height 480 \
        --square-size-mm 3.0 --out-dir calib_frames_fisheye/
    python3 calibrate_camera_fisheye.py calibrate --frames-dir calib_frames_fisheye/ \
        --square-size-mm 3.0 --out camera_info_fisheye.yaml

--square-size-mm and board-size defaults are the same as calibrate_camera.py
(see its docstring) -- same physical board, same care about matching
whatever you actually printed.

Capture technique differs from hand-eye capture in one important way: for
*this* step (fitting the distortion model itself), deliberately include
views where the board reaches toward the edges/corners of the frame, not
just centred ones -- the k3/k4 fisheye terms are only well constrained by
corners sampled out at the extreme radii where the distortion is largest.
(This is the opposite of the advice for hand-eye capture, where a
comfortably-centred board is preferred once intrinsics are already known --
those are two different steps with different sampling needs.)
"""

import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np
import yaml

from calibrate_camera import cmd_capture
from calibration_common import find_corners


def _fisheye_flag(name):
    """CALIB_* flag constants live under cv2.fisheye.* on some OpenCV
    builds and only at the top-level cv2.* on others (confirmed by hand
    against two different installed builds while developing this script) --
    look in both places rather than hardcoding one."""
    if hasattr(cv2.fisheye, name):
        return getattr(cv2.fisheye, name)
    return getattr(cv2, name)


def cmd_calibrate(args):
    paths = sorted(glob.glob(os.path.join(args.frames_dir, '*.png')))
    if not paths:
        print(f'No .png frames found in {args.frames_dir}', file=sys.stderr)
        sys.exit(1)

    objp = np.zeros((args.corners_x * args.corners_y, 1, 3), np.float64)
    objp[:, 0, :2] = np.mgrid[0:args.corners_x, 0:args.corners_y].T.reshape(-1, 2)
    objp *= args.square_size_mm

    objpoints = []
    imgpoints = []
    frame_paths_used = []
    image_size = None
    skipped = 0

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
        # cv2.fisheye.calibrate is stricter about dtype than
        # cv2.calibrateCamera -- validated against a real OpenCV 4.9.0
        # build that float64 (N,1,3)/(N,1,2) per view is what it wants.
        objpoints.append(objp.copy())
        imgpoints.append(corners.astype(np.float64))
        frame_paths_used.append(path)

    if len(objpoints) < 10:
        print(f'Only {len(objpoints)} usable frames (skipped {skipped}) -- capture more '
              'before trusting this calibration.', file=sys.stderr)

    flags = (
        _fisheye_flag('CALIB_RECOMPUTE_EXTRINSIC')
        | _fisheye_flag('CALIB_CHECK_COND')
        | _fisheye_flag('CALIB_FIX_SKEW')
    )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)

    # cv2.fisheye.calibrate's CALIB_CHECK_COND flag raises rather than just
    # down-weighting a single ill-conditioned view, and names which one in
    # its error message ("...Ill-conditioned matrix for input array N...")
    # -- confirmed against a real OpenCV 4.9.0 build while developing this
    # script. Drop that one view and retry instead of failing the whole
    # calibration outright, same spirit as skipping frames with no board.
    dropped_ill_conditioned = []
    while True:
        if len(objpoints) < 4:
            print('Too few views left after dropping ill-conditioned ones -- '
                  'capture more, covering a wider range of distances/angles.',
                  file=sys.stderr)
            sys.exit(1)
        try:
            rms, camera_matrix, dist_coeffs, _, _ = cv2.fisheye.calibrate(
                objpoints, imgpoints, image_size,
                np.eye(3), np.zeros((4, 1)),
                flags=flags, criteria=criteria,
            )
            break
        except cv2.error as exc:
            match = re.search(r'input array (\d+)', str(exc))
            if match is None:
                print(f'cv2.fisheye.calibrate failed: {exc}', file=sys.stderr)
                sys.exit(1)
            bad_index = int(match.group(1))
            if bad_index >= len(objpoints):
                print(f'cv2.fisheye.calibrate failed and named an out-of-range '
                      f'view index ({bad_index}): {exc}', file=sys.stderr)
                sys.exit(1)
            dropped_ill_conditioned.append(frame_paths_used.pop(bad_index))
            objpoints.pop(bad_index)
            imgpoints.pop(bad_index)
            print(f'  view {bad_index} ({dropped_ill_conditioned[-1]}) is '
                  f'ill-conditioned (CALIB_CHECK_COND) -- dropping it and '
                  f'retrying ({len(objpoints)} views left)')

    if dropped_ill_conditioned:
        print(f'Dropped {len(dropped_ill_conditioned)} ill-conditioned view(s): '
              f'{dropped_ill_conditioned}')
    print(f'Used {len(objpoints)}/{len(paths)} frames, skipped {skipped} (no board found)')
    print(f'RMS reprojection error: {rms:.4f} px')
    if rms > 1.0:
        print('  >1px is high for this image size -- check the board is flat, '
              'square-size-mm is correct, and frames cover both the centre '
              'and the edges/corners of the frame (needed to constrain k3/k4).')
    print('Camera matrix (K):')
    print(camera_matrix)
    print('Distortion coefficients (k1 k2 k3 k4, fisheye/equidistant model):')
    print(dist_coeffs.ravel())

    # Same ROS sensor_msgs/CameraInfo-compatible layout calibrate_camera.py
    # writes -- camera_node.py copies distortion_model/d/k/r/p through
    # verbatim regardless of value, and every other consumer branches on
    # distortion_model, so this is a drop-in alternative output.
    camera_info = {
        'image_width': image_size[0],
        'image_height': image_size[1],
        'camera_name': 'naneye',
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': camera_matrix.flatten().tolist(),
        },
        'distortion_model': 'equidistant',
        'distortion_coefficients': {
            'rows': 1, 'cols': 4,
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
    p_capture.add_argument('--crop-x', type=int, default=0)
    p_capture.add_argument('--crop-y', type=int, default=0)
    p_capture.add_argument('--crop-width', type=int, default=0)
    p_capture.add_argument('--crop-height', type=int, default=0)
    p_capture.add_argument('--out-dir', default='calib_frames_fisheye')
    p_capture.set_defaults(func=cmd_capture)

    p_calibrate = sub.add_parser('calibrate', parents=[common])
    p_calibrate.add_argument('--frames-dir', default='calib_frames_fisheye')
    p_calibrate.add_argument('--out', default='camera_info_fisheye.yaml')
    p_calibrate.set_defaults(func=cmd_calibrate)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
