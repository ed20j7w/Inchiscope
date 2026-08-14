#!/usr/bin/env python3
"""Hand-eye calibration: solves for the fixed rigid transform between the
Aurora 6D EM sensor and the NanEye camera, both mounted together at the
endoscope's distal tip. Replaces the zero-offset PLACEHOLDER static
transform in camera_and_aurora.launch.py / inchiscope.launch.py
(aurora_sensor_0 -> naneye_camera) with the real measured offset.

Two subcommands:
    capture     ROS2 node: live view against /camera/image_raw with a
                checkerboard overlay; SPACE grabs an (image, Aurora pose)
                pair when the board is detected AND a fresh pose is
                available, 'q' finishes. Saves frames + a poses.yaml
                manifest into --out-dir.
    solve       Offline OpenCV/numpy: runs cv2.calibrateHandEye() over the
                captures, cross-checks all 5 supported methods for
                agreement, validates by checking how consistent the
                inferred (fixed) checkerboard-in-reference-frame pose is
                across captures, and writes hand_eye_transform.yaml plus
                the exact static_transform_publisher arguments to paste
                into the launch files.

Usage:
    # 1. Launch camera + Aurora together (needed for /camera/image_raw and
    #    /aurora/sensor_0/pose_relative_to_reference to both be publishing):
    #        ros2 launch inchiscope_bringup camera_and_aurora.launch.py

    # 2. Hold the checkerboard FIXED and STATIONARY somewhere in view, then
    #    move the endoscope tip (camera+sensor assembly) by hand to 15-20+
    #    diverse poses, capturing at each:
    python3 calibrate_hand_eye.py capture --square-size-mm 3.0 --out-dir handeye_frames/

    # 3. Solve:
    python3 calibrate_hand_eye.py solve --frames-dir handeye_frames/ \
        --square-size-mm 3.0 --camera-info ../../inchiscope_bringup/config/camera_info.yaml \
        --out hand_eye_transform.yaml

Physical procedure -- read before capturing:
- The checkerboard must not move at all during the whole session. Every
  capture is treated as "the same fixed checkerboard, seen from a
  different camera pose" -- if the board moves between captures, the math
  silently assumes it didn't and the solve will be wrong with no obvious
  symptom other than a large residual in `solve`'s validation step.
- The camera+Aurora-sensor assembly (endoscope tip) is what moves. Move it
  through a real range of *rotations*, not just translations/slides --
  translation-only motion leaves the rotation part of AX=XB poorly
  constrained (a known degeneracy of hand-eye calibration), even though
  the board may still look "detected fine" in every frame.
- 15-20+ good captures is typical, matching the corner-detection/pose
  variety guidance already used for `calibrate_camera.py`.
- Board size/corner count defaults match `calibrate_camera.py` and
  `generate_calibration_target.py` (9x6 internal corners) -- override with
  --corners-x/--corners-y if a different board was printed, and
  --square-size-mm must match whichever printed size was actually used.

Math convention (see the module docstring of `solve_hand_eye` below for the
full derivation): Aurora's `/aurora/.../pose_relative_to_reference` maps
directly onto OpenCV's `R_gripper2base`/`t_gripper2base` (no inversion
needed), and `cv2.solvePnP`'s checkerboard-in-camera-frame output maps
directly onto `R_target2cam`/`t_target2cam`. The transform
`cv2.calibrateHandEye` then returns (`R_cam2gripper`/`t_cam2gripper`) is
exactly "camera pose expressed in the Aurora sensor frame" -- i.e. the TF
transform with `--frame-id aurora_sensor_0 --child-frame-id naneye_camera`.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

from calibrate_camera import find_corners


def quat_to_rotmat(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rotmat_to_quat(R):
    """Shepperd's method -- numerically stable for all rotations, unlike the
    naive formula which divides by ~0 near a 180-degree rotation."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([x, y, z, w])


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cmd_capture(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge

    class HandEyeCaptureNode(Node):
        def __init__(self):
            super().__init__('calibrate_hand_eye_capture')
            self.bridge = CvBridge()
            self.latest_image = None
            self.latest_image_stamp = None
            self.latest_pose = None
            self.latest_pose_stamp = None
            self.create_subscription(Image, '/camera/image_raw', self._on_image, 10)
            self.create_subscription(PoseStamped, args.pose_topic, self._on_pose, 10)

        def _on_image(self, msg):
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_stamp = msg.header.stamp

        def _on_pose(self, msg):
            self.latest_pose = msg.pose
            self.latest_pose_stamp = msg.header.stamp

    os.makedirs(args.out_dir, exist_ok=True)
    rclpy.init()
    node = HandEyeCaptureNode()
    captures = []
    saved = 0
    print("Checkerboard must stay FIXED for the whole session -- only move "
          "the camera+Aurora-sensor assembly. SPACE = capture when the "
          "board is detected (drawn in colour) and the pose reading is "
          "fresh, q = finish. Cover a real range of rotation, not just "
          "translation -- 15-20 good captures is typical.")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.latest_image is None:
                continue
            frame = node.latest_image.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = find_corners(gray, args.corners_x, args.corners_y)
            display = frame.copy()
            if found:
                cv2.drawChessboardCorners(display, (args.corners_x, args.corners_y), corners, found)

            pose_age = None
            if node.latest_pose_stamp is not None and node.latest_image_stamp is not None:
                pose_age = abs(stamp_to_sec(node.latest_image_stamp) - stamp_to_sec(node.latest_pose_stamp))

            status = f'saved: {saved}'
            status += '  NO POSE' if pose_age is None else f'  pose_age={pose_age * 1000:.0f}ms'
            colour = (0, 255, 0) if found else (0, 0, 255)
            cv2.putText(display, status, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)
            cv2.imshow('calibrate_hand_eye - capture', display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                if not found:
                    print('  no checkerboard detected, not captured')
                    continue
                if node.latest_pose is None:
                    print('  no Aurora pose received yet, not captured')
                    continue
                if pose_age is None or pose_age > args.max_pose_age_sec:
                    age_str = 'unknown' if pose_age is None else f'{pose_age:.3f}s'
                    print(f'  pose too stale ({age_str} > {args.max_pose_age_sec}s), not captured')
                    continue
                fname = f'frame_{saved:03d}.png'
                cv2.imwrite(os.path.join(args.out_dir, fname), frame)
                p = node.latest_pose.position
                q = node.latest_pose.orientation
                captures.append({
                    'frame': fname,
                    'position': [float(p.x), float(p.y), float(p.z)],
                    'orientation_xyzw': [float(q.x), float(q.y), float(q.z), float(q.w)],
                })
                saved += 1
                print(f'captured {fname} (pose_age={pose_age * 1000:.0f}ms)')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    with open(os.path.join(args.out_dir, 'poses.yaml'), 'w') as f:
        yaml.safe_dump({'captures': captures}, f, sort_keys=False)
    print(f'Captured {saved} image+pose pairs to {args.out_dir}')
    if saved < 15:
        print('Fewer than 15 captures -- the solve will likely be poorly '
              'constrained, especially rotation. Aim for 15-20+, spanning '
              'a real range of tip rotation, not just translation.')


def load_camera_info(path):
    with open(path) as f:
        info = yaml.safe_load(f)
    K = np.array(info['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(info['distortion_coefficients']['data'], dtype=np.float64)
    return K, D, info['image_width'], info['image_height']


HAND_EYE_METHODS = [
    ('TSAI', cv2.CALIB_HAND_EYE_TSAI),
    ('PARK', cv2.CALIB_HAND_EYE_PARK),
    ('HORAUD', cv2.CALIB_HAND_EYE_HORAUD),
    ('ANDREFF', cv2.CALIB_HAND_EYE_ANDREFF),
    ('DANIILIDIS', cv2.CALIB_HAND_EYE_DANIILIDIS),
]


def cmd_solve(args):
    if not hasattr(cv2, 'calibrateHandEye'):
        print("This OpenCV build/install is missing cv2.calibrateHandEye() "
              "(present in opencv-python's calib3d module since 4.1.0 -- "
              "some minimal/headless builds have been seen to omit it). "
              "Install/upgrade a standard opencv-python(-headless) wheel "
              "and retry.", file=sys.stderr)
        sys.exit(1)

    with open(os.path.join(args.frames_dir, 'poses.yaml')) as f:
        manifest = yaml.safe_load(f)
    captures = manifest['captures']
    if len(captures) < 3:
        print('Need at least 3 captures to solve (15-20+ recommended).', file=sys.stderr)
        sys.exit(1)

    K, D, img_w, img_h = load_camera_info(args.camera_info)

    objp = np.zeros((args.corners_x * args.corners_y, 3), np.float64)
    objp[:, :2] = np.mgrid[0:args.corners_x, 0:args.corners_y].T.reshape(-1, 2)
    objp *= args.square_size_mm / 1000.0  # mm -> m, to match Aurora's metres

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []
    used, skipped = 0, 0

    for cap in captures:
        path = os.path.join(args.frames_dir, cap['frame'])
        img = cv2.imread(path)
        if img is None:
            print(f"  couldn't read {path}, skipping")
            skipped += 1
            continue
        if img.shape[1] != img_w or img.shape[0] != img_h:
            print(f"  {path} is {img.shape[1]}x{img.shape[0]}, camera-info is "
                  f"{img_w}x{img_h} -- mismatch, skipping")
            skipped += 1
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = find_corners(gray, args.corners_x, args.corners_y)
        if not found:
            print(f'  no board found in {path}, skipping')
            skipped += 1
            continue
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
        if not ok:
            print(f'  solvePnP failed for {path}, skipping')
            skipped += 1
            continue
        R_t2c, _ = cv2.Rodrigues(rvec)
        R_target2cam.append(R_t2c)
        t_target2cam.append(tvec.ravel())

        R_gripper2base.append(quat_to_rotmat(*cap['orientation_xyzw']))
        t_gripper2base.append(np.array(cap['position'], dtype=np.float64))
        used += 1

    print(f'Used {used}/{len(captures)} captures, skipped {skipped}')
    if used < 3:
        print('Fewer than 3 usable captures -- cannot solve.', file=sys.stderr)
        sys.exit(1)

    def checkerboard_pose_spread(R_x, t_x):
        """Since the checkerboard is physically fixed relative to the
        reference frame all session, T_base2target_i should come out
        (near-)identical for every capture if the solved hand-eye
        transform (R_x, t_x = R/t_cam2gripper) is correct. Its spread
        across captures is therefore a direct validation metric -- more
        diagnostic for hand-eye specifically than reprojection error alone,
        which mainly re-validates the earlier intrinsic calibration/PnP
        step."""
        positions = []
        for i in range(used):
            T_base2gripper = np.eye(4)
            T_base2gripper[:3, :3] = R_gripper2base[i]
            T_base2gripper[:3, 3] = t_gripper2base[i]
            T_cam2gripper = np.eye(4)
            T_cam2gripper[:3, :3] = R_x
            T_cam2gripper[:3, 3] = t_x
            T_target2cam = np.eye(4)
            T_target2cam[:3, :3] = R_target2cam[i]
            T_target2cam[:3, 3] = t_target2cam[i]
            T_base2target = T_base2gripper @ T_cam2gripper @ T_target2cam
            positions.append(T_base2target[:3, 3])
        positions = np.array(positions)
        return positions.std(axis=0), (positions.max(0) - positions.min(0))

    print()
    print('Cross-checking all 5 calibrateHandEye methods (should roughly '
          'agree if the capture set is good):')
    results = {}
    for name, method in HAND_EYE_METHODS:
        R_x, t_x = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=method,
        )
        t_x = t_x.ravel()
        std, spread = checkerboard_pose_spread(R_x, t_x)
        results[name] = (R_x, t_x, std, spread)
        print(f'  {name:12s} checkerboard-in-reference std={std * 1000} mm  '
              f'max-min={spread * 1000} mm')

    R_final, t_final, std_final, spread_final = results[args.method]
    print()
    print(f'Using {args.method} as the final result.')
    if np.max(spread_final) * 1000 > args.max_spread_warn_mm:
        print(f'WARNING: checkerboard-in-reference spread exceeds '
              f'{args.max_spread_warn_mm}mm (max axis '
              f'{np.max(spread_final) * 1000:.2f}mm) -- this usually means '
              f'the checkerboard moved during capture, too few/too '
              f'rotation-poor poses, or a bad pose/frame pairing. Re-capture '
              f'before trusting this result.', file=sys.stderr)

    quat = rotmat_to_quat(R_final)
    out = {
        'parent_frame': 'aurora_sensor_0',
        'child_frame': 'naneye_camera',
        'translation': {'x': float(t_final[0]), 'y': float(t_final[1]), 'z': float(t_final[2])},
        'rotation_xyzw': {'x': float(quat[0]), 'y': float(quat[1]), 'z': float(quat[2]), 'w': float(quat[3])},
        'method': args.method,
        'captures_used': used,
        'validation_std_mm': [float(v) for v in (std_final * 1000)],
        'validation_max_spread_mm': [float(v) for v in (spread_final * 1000)],
    }
    with open(args.out, 'w') as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False)
    print(f'Wrote {args.out}')
    print()
    print('Paste into camera_and_aurora.launch.py / inchiscope.launch.py, '
          'replacing the zero-offset PLACEHOLDER static_transform_publisher '
          'arguments:')
    print(f"    '--x', '{t_final[0]:.6f}', '--y', '{t_final[1]:.6f}', '--z', '{t_final[2]:.6f}',")
    print(f"    '--qx', '{quat[0]:.6f}', '--qy', '{quat[1]:.6f}', '--qz', '{quat[2]:.6f}', '--qw', '{quat[3]:.6f}',")
    print("    '--frame-id', 'aurora_sensor_0',")
    print("    '--child-frame-id', 'naneye_camera',")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--corners-x', type=int, default=9, help='internal corners, horizontal (default: 9, matches generate_calibration_target.py)')
    common.add_argument('--corners-y', type=int, default=6, help='internal corners, vertical (default: 6)')
    common.add_argument('--square-size-mm', type=float, required=True, help='real-world size of one printed square, in mm -- must match the board actually used')

    p_capture = sub.add_parser('capture', parents=[common])
    p_capture.add_argument('--pose-topic', default='/aurora/sensor_0/pose_relative_to_reference')
    p_capture.add_argument('--max-pose-age-sec', type=float, default=0.1, help='reject a capture if the freshest Aurora pose is older than this relative to the image (default: 0.1s)')
    p_capture.add_argument('--out-dir', default='handeye_frames')
    p_capture.set_defaults(func=cmd_capture)

    p_solve = sub.add_parser('solve', parents=[common])
    p_solve.add_argument('--frames-dir', default='handeye_frames')
    p_solve.add_argument('--camera-info', required=True, help='path to camera_info.yaml (e.g. src/inchiscope_bringup/config/camera_info.yaml)')
    p_solve.add_argument('--method', choices=[n for n, _ in HAND_EYE_METHODS], default='TSAI')
    p_solve.add_argument('--max-spread-warn-mm', type=float, default=2.0, help='warn if the checkerboard-in-reference-frame validation spread exceeds this many mm on any axis (default: 2.0)')
    p_solve.add_argument('--out', default='hand_eye_transform.yaml')
    p_solve.set_defaults(func=cmd_solve)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
