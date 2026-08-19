#!/usr/bin/env python3
"""Standalone CLI running Stage 1 (extract & associate) + Stage 2 (frame
selection) against a real rosbag, reporting what happened at each step.
Lets this be tested end-to-end against real hardware data before
reconstruction_node/the ReconstructFromBag action exist (milestone 2 of
the reconstruction workplan) -- run it, check the counts/warnings below
look sane, then we move on to Stage 3.

Usage:
    ros2 run inchiscope_reconstruction extract_and_select \\
        --bag-path /path/to/bag \\
        --camera-info src/inchiscope_bringup/config/camera_info.yaml \\
        --hand-eye src/inchiscope_camera/scripts/hand_eye_transform.yaml \\
        --out-dir stage2_frames/
"""

import argparse
import os
import sys

import cv2
import numpy as np

from inchiscope_reconstruction.calibration import load_camera_intrinsics, load_hand_eye_transform
from inchiscope_reconstruction.bag_extraction import (
    IMAGE_TOPIC, POSE_TOPIC, read_bag, associate, apply_hand_eye,
)
from inchiscope_reconstruction.frame_selection import blur_scores, select_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bag-path', required=True)
    parser.add_argument('--camera-info', required=True, help='e.g. src/inchiscope_bringup/config/camera_info.yaml')
    parser.add_argument('--hand-eye', required=True, help='e.g. src/inchiscope_camera/scripts/hand_eye_transform.yaml')
    parser.add_argument('--image-topic', default=IMAGE_TOPIC)
    parser.add_argument('--pose-topic', default=POSE_TOPIC)
    parser.add_argument('--max-pose-age-sec', type=float, default=0.1)
    parser.add_argument('--blur-threshold', type=float, default=100.0, help='variance-of-Laplacian cutoff, drop frames below it (default: 100.0 -- tune after looking at real footage)')
    parser.add_argument('--min-baseline-m', type=float, default=0.001, help='minimum camera translation from the last kept frame to avoid being dropped as redundant, in metres (default: 1mm)')
    parser.add_argument('--min-rotation-deg', type=float, default=2.0, help='minimum camera rotation from the last kept frame to avoid being dropped as redundant, in degrees (default: 2.0)')
    parser.add_argument('--out-dir', default=None, help='if set, writes the kept, undistorted frames here as PNGs for visual inspection')
    args = parser.parse_args()

    K, D, distortion_model, img_w, img_h = load_camera_intrinsics(args.camera_info)
    R_ce, t_ce = load_hand_eye_transform(args.hand_eye)

    images, poses = read_bag(args.bag_path, args.image_topic, args.pose_topic)
    print(f'Read {len(images)} images, {len(poses)} poses from {args.bag_path}')
    if not images:
        print('No images found -- check --image-topic and that the bag actually recorded it.', file=sys.stderr)
        sys.exit(1)
    if images[0][1].shape[1] != img_w or images[0][1].shape[0] != img_h:
        print(f'WARNING: bag images are {images[0][1].shape[1]}x{images[0][1].shape[0]}, '
              f'camera-info says {img_w}x{img_h} -- mismatch, undistort() below will be wrong. '
              f'Was this bag recorded at a different camera_node resolution/crop than the current '
              f'camera_info.yaml was calibrated against?', file=sys.stderr)

    associated, dropped_stale = associate(images, poses, args.max_pose_age_sec)
    print(f'Stage 1: associated {len(associated)}/{len(images)} images to a pose within '
          f'{args.max_pose_age_sec}s ({dropped_stale} dropped as stale)')
    if associated:
        ages_ms = [a[4] * 1000 for a in associated]
        print(f'  pose age: min={min(ages_ms):.1f}ms max={max(ages_ms):.1f}ms '
              f'mean={sum(ages_ms) / len(ages_ms):.1f}ms')

    with_poses = apply_hand_eye(associated, R_ce, t_ce)

    scores = blur_scores(with_poses)
    pct = np.percentile(scores, [0, 10, 25, 50, 75, 90, 100])
    print(f'Blur score distribution (variance of Laplacian, higher=sharper) over '
          f'{len(scores)} frames:')
    print(f'  min={pct[0]:.1f} p10={pct[1]:.1f} p25={pct[2]:.1f} median={pct[3]:.1f} '
          f'p75={pct[4]:.1f} p90={pct[5]:.1f} max={pct[6]:.1f}  '
          f'(current --blur-threshold={args.blur_threshold})')
    if pct[6] < args.blur_threshold:
        print(f'  NOTE: every frame scores below --blur-threshold, so Stage 2 will drop all of '
              f'them regardless of --min-baseline-m/--min-rotation-deg. This metric reads '
              f'low-texture, smooth, evenly-lit content (e.g. mucosa) as "blurry" even in '
              f'perfect focus -- there is no universal good threshold, so pick one relative to '
              f'*this* distribution (e.g. drop only the bottom 10-25% via p10/p25 above) rather '
              f'than an absolute number, then look at a few --out-dir frames near that cutoff to '
              f'confirm they actually look unusably blurry before trusting it.', file=sys.stderr)

    kept, dropped_blur, dropped_redundant = select_frames(
        with_poses, K, D,
        blur_threshold=args.blur_threshold,
        min_baseline_m=args.min_baseline_m,
        min_rotation_deg=args.min_rotation_deg,
        distortion_model=distortion_model,
    )
    print(f'Stage 2: kept {len(kept)}/{len(with_poses)} frames '
          f'({dropped_blur} dropped as blurry, {dropped_redundant} dropped as redundant)')
    if len(kept) < 10:
        print('Fewer than 10 frames survived -- too aggressive thresholds, or a genuinely '
              'short/static capture? Check --blur-threshold/--min-baseline-m/--min-rotation-deg '
              'before trusting downstream stages with this few views.', file=sys.stderr)

    if args.out_dir:
        os.makedirs(args.out_dir, exist_ok=True)
        for i, (t_sec, img, T_ref2cam, pose_age) in enumerate(kept):
            cv2.imwrite(os.path.join(args.out_dir, f'frame_{i:04d}.png'), img)
        print(f'Wrote {len(kept)} undistorted frames to {args.out_dir}')


if __name__ == '__main__':
    main()
