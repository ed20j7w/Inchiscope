#!/usr/bin/env python3
"""Standalone CLI running the full Stage 1 -> 2 -> 3 pipeline against a
real rosbag: extract & associate, frame selection, then the sparse
sanity check (SIFT matching + triangulation with known, fixed poses).
Purely to catch a bad hand-eye calibration or corrupted pose stream
before paying for COLMAP's dense stereo (milestone 3 of the
reconstruction workplan).

Re-runs Stage 1-2 itself rather than reading extract_and_select's
--out-dir PNGs back in, since those don't carry the per-frame poses Stage
3 needs -- Stage 1-2 are cheap, so this is simpler than adding a pose
serialization format just to pass data between two CLIs.

Usage:
    ros2 run inchiscope_reconstruction sparse_sanity_check \\
        --bag-path /path/to/bag \\
        --camera-info src/inchiscope_bringup/config/camera_info.yaml \\
        --hand-eye src/inchiscope_camera/scripts/hand_eye_transform.yaml \\
        --expected-diameter-mm 25.0
"""

import argparse
import sys

from inchiscope_reconstruction.calibration import load_camera_intrinsics, load_hand_eye_transform
from inchiscope_reconstruction.bag_extraction import (
    IMAGE_TOPIC, POSE_TOPIC, read_bag, associate, apply_hand_eye,
)
from inchiscope_reconstruction.frame_selection import select_frames
from inchiscope_reconstruction.sanity_check import sparse_sanity_check, scale_plausibility


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bag-path', required=True)
    parser.add_argument('--camera-info', required=True)
    parser.add_argument('--hand-eye', required=True)
    parser.add_argument('--image-topic', default=IMAGE_TOPIC)
    parser.add_argument('--pose-topic', default=POSE_TOPIC)
    parser.add_argument('--max-pose-age-sec', type=float, default=0.1)
    parser.add_argument('--blur-threshold', type=float, default=0.0, help='Stage 2 blur cutoff (default: 0.0, i.e. disabled -- see inchiscope_reconstruction/README.md for why variance-of-Laplacian may not be useful on your footage)')
    parser.add_argument('--min-baseline-m', type=float, default=0.001)
    parser.add_argument('--min-rotation-deg', type=float, default=2.0)
    parser.add_argument('--pair-stride', type=int, default=5, help='match/triangulate frames this many Stage-2-kept-frames apart, for a wider baseline than adjacent frames (default: 5)')
    parser.add_argument('--ratio', type=float, default=0.75, help="Lowe's ratio test threshold for SIFT matching (default: 0.75)")
    parser.add_argument('--max-epipolar-error-px', type=float, default=3.0, help='discard matches whose epipolar error (against the fundamental matrix from the KNOWN poses) exceeds this many px (default: 3.0)')
    parser.add_argument('--expected-diameter-mm', type=float, required=True, help='expected scene scale (e.g. lumen diameter) in mm, used only as a coarse order-of-magnitude plausibility bound')
    parser.add_argument('--tolerance-factor', type=float, default=5.0, help='triangulated scale is flagged implausible outside [expected/factor, expected*factor] (default: 5.0)')
    args = parser.parse_args()

    K, D, img_w, img_h = load_camera_intrinsics(args.camera_info)
    R_ce, t_ce = load_hand_eye_transform(args.hand_eye)

    images, poses = read_bag(args.bag_path, args.image_topic, args.pose_topic)
    print(f'Read {len(images)} images, {len(poses)} poses from {args.bag_path}')
    if not images:
        print('No images found.', file=sys.stderr)
        sys.exit(1)

    associated, dropped_stale = associate(images, poses, args.max_pose_age_sec)
    with_poses = apply_hand_eye(associated, R_ce, t_ce)
    kept, dropped_blur, dropped_redundant = select_frames(
        with_poses, K, D,
        blur_threshold=args.blur_threshold,
        min_baseline_m=args.min_baseline_m,
        min_rotation_deg=args.min_rotation_deg,
    )
    print(f'Stage 1-2: {len(images)} images -> {len(associated)} associated -> {len(kept)} kept '
          f'({dropped_stale} stale, {dropped_blur} blurry, {dropped_redundant} redundant)')
    if len(kept) < args.pair_stride + 1:
        print(f'Fewer than {args.pair_stride + 1} frames survived Stage 2 -- not enough to form '
              f'even one --pair-stride={args.pair_stride} pair. Lower --pair-stride or loosen '
              f'Stage 2 thresholds.', file=sys.stderr)
        sys.exit(1)

    result = sparse_sanity_check(
        kept, K,
        pair_stride=args.pair_stride,
        ratio=args.ratio,
        max_epipolar_error_px=args.max_epipolar_error_px,
    )
    if result is None:
        print('Stage 3: no frame pair produced enough surviving matches to triangulate anything. '
              'This itself is a bad sign -- either the content has too little texture for SIFT to '
              'find matches at all, --pair-stride is too large (baseline too wide for matching to '
              'survive), or the pose stream/hand-eye transform is wrong enough that even correct '
              'matches keep failing the epipolar check.', file=sys.stderr)
        sys.exit(1)

    print(f"Stage 3: {result['pair_count']} frame pairs produced matches, "
          f"{result['point_count']} points triangulated total "
          f"(per-pair counts: {result['match_counts']})")
    print(f"  reprojection error (px): mean={result['reproj_error_mean_px']:.2f} "
          f"median={result['reproj_error_median_px']:.2f} p90={result['reproj_error_p90_px']:.2f}")
    if result['reproj_error_median_px'] > 5.0:
        print('  WARNING: median reprojection error is high for triangulation against known, fixed '
              'poses (should generally be a few px at most) -- suggests the pose stream/hand-eye '
              'transform may be off even where matches survived the epipolar filter.', file=sys.stderr)

    plausibility = scale_plausibility(result['points'], args.expected_diameter_mm, args.tolerance_factor)
    lo, hi = plausibility['bounds_mm']
    print(f"  triangulated scene extent (mm, 5th-95th percentile per axis): "
          f"{[round(v, 1) for v in plausibility['extent_mm']]}")
    print(f"  characteristic size: {plausibility['characteristic_size_mm']:.1f}mm "
          f"(plausible range given expected {args.expected_diameter_mm}mm +/- {args.tolerance_factor}x: "
          f"{lo:.1f}-{hi:.1f}mm)")
    if plausibility['plausible']:
        print('  PASS: triangulated scale is plausible -- pose stream and hand-eye transform look '
              'directionally sane. This is a coarse bound, not proof of accuracy.')
    else:
        print('  FAIL: triangulated scale is NOT plausible -- do not proceed to dense stereo yet. '
              'Suspect the hand-eye transform, a corrupted pose stream, or (per project discussion) '
              'camera/Aurora timestamp mismatch during fast motion.', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
