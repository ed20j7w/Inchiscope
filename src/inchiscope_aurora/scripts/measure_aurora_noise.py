#!/usr/bin/env python3
"""Measures the Aurora tracker's own noise floor: hold the sensor
completely still and this reports how much
/aurora/sensor_0/pose_relative_to_reference jitters on its own, in
position (mm) and orientation (degrees).

This isolates the tracker's own accuracy from everything else in the
pipeline (capture timing, hand-eye calibration, camera calibration) --
it's a direct, independent measurement to compare against
calibrate_hand_eye.py solve's checkerboard-in-reference spread. If the
numbers here are already close to that spread, no amount of better
capture technique or recalibration elsewhere can beat it -- it's a hard
floor set by the tracker/environment (nearby metal, distance from the
field generator) at the bench location tested.

Usage:
    # With camera_and_aurora.launch.py (or just aurora.launch.py) already
    # running and the sensor held as still as possible:
    python3 measure_aurora_noise.py --duration-sec 15
"""

import argparse
import sys
import time

import numpy as np


def quat_to_rotmat(x, y, z, w):
    """Same formula as inchiscope_camera/scripts/calibrate_hand_eye.py's
    helper of the same name (validated there against 2000 random rotations
    to machine precision)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rotation_angle_deg(R):
    """Angle of a rotation matrix, in degrees, via the standard trace
    formula (angle = arccos((trace(R) - 1) / 2)), clipped for float
    safety near +-1."""
    val = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(val)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pose-topic', default='/aurora/sensor_0/pose_relative_to_reference')
    parser.add_argument('--duration-sec', type=float, default=15.0, help='how long to collect samples for (default: 15s)')
    parser.add_argument('--settle-sec', type=float, default=1.0, help='discard this many seconds at the start, to let any initial motion/settling pass (default: 1.0s)')
    args = parser.parse_args()  # handles --help/-h and exits before ever touching rclpy

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped

    class NoiseNode(Node):
        def __init__(self):
            super().__init__('measure_aurora_noise')
            self.samples = []  # (t_monotonic, position_xyz, quat_xyzw)
            self.create_subscription(PoseStamped, args.pose_topic, self._on_pose, 50)

        def _on_pose(self, msg):
            p = msg.pose.position
            q = msg.pose.orientation
            self.samples.append((
                time.monotonic(),
                np.array([p.x, p.y, p.z]),
                np.array([q.x, q.y, q.z, q.w]),
            ))

    rclpy.init()
    node = NoiseNode()
    print(f'Hold the sensor completely still -- resting it on something solid works better '
          f'than free-handing it. Collecting for {args.duration_sec:.0f}s (first '
          f'{args.settle_sec:.1f}s discarded to let any initial motion settle)...')
    start = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - start) < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    kept = [(t, p, q) for (t, p, q) in node.samples if t - start >= args.settle_sec]
    print(f'Collected {len(node.samples)} samples total, {len(kept)} after discarding the settle period')
    if len(kept) < 10:
        print('Too few samples -- check the topic is actually publishing '
              '(camera_and_aurora.launch.py or aurora.launch.py running?) and that '
              '--duration-sec is long enough.', file=sys.stderr)
        sys.exit(1)

    times = np.array([t for t, _, _ in kept])
    positions = np.array([p for _, p, _ in kept])
    quats = np.array([q for _, _, q in kept])

    dt = np.diff(times)
    rate_hz = 1.0 / np.median(dt) if len(dt) else float('nan')

    pos_mean = positions.mean(axis=0)
    pos_std_mm = positions.std(axis=0) * 1000.0
    pos_ptp_mm = (positions.max(axis=0) - positions.min(axis=0)) * 1000.0
    pos_dev_from_mean_mm = np.linalg.norm(positions - pos_mean, axis=1) * 1000.0

    # Orientation jitter relative to the middle sample (avoids needing a
    # proper quaternion-averaging algorithm just to characterise spread;
    # validated synthetically to give sensible, if slightly conservative,
    # numbers against a known-injected noise level).
    ref_idx = len(quats) // 2
    R_ref = quat_to_rotmat(*quats[ref_idx])
    angles_deg = np.array([
        rotation_angle_deg(R_ref.T @ quat_to_rotmat(*q)) for q in quats
    ])

    print()
    print(f'Sample rate observed: {rate_hz:.1f} Hz over {len(kept)} samples '
          f'({times[-1] - times[0]:.1f}s span)')
    print()
    print('Position jitter (mm), while held still:')
    print(f'  per-axis std:          x={pos_std_mm[0]:.3f}  y={pos_std_mm[1]:.3f}  z={pos_std_mm[2]:.3f}')
    print(f'  per-axis peak-to-peak: x={pos_ptp_mm[0]:.3f}  y={pos_ptp_mm[1]:.3f}  z={pos_ptp_mm[2]:.3f}')
    print(f'  3D deviation from mean position: mean={pos_dev_from_mean_mm.mean():.3f}mm  '
          f'median={np.median(pos_dev_from_mean_mm):.3f}mm  max={pos_dev_from_mean_mm.max():.3f}mm')
    print()
    print('Orientation jitter (deg), while held still (relative to the middle sample):')
    print(f'  std={angles_deg.std():.3f}deg  median={np.median(angles_deg):.3f}deg  max={angles_deg.max():.3f}deg')
    print()
    print('For context: this noise floor propagates directly into hand-eye calibration and '
          'reconstruction pose accuracy -- it is a hard lower bound no amount of capture '
          'technique or recalibration elsewhere can beat. Compare these numbers directly '
          'against the checkerboard-in-reference spread calibrate_hand_eye.py solve reports -- '
          'if they are already close, the tracker/environment (not your capture technique) is '
          'the limiting factor at this bench location.')


if __name__ == '__main__':
    main()
