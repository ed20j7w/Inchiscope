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
import json
import sys

import numpy as np

from inchiscope_reconstruction.calibration import load_camera_intrinsics, load_hand_eye_transform
from inchiscope_reconstruction.bag_extraction import (
    IMAGE_TOPIC, POSE_TOPIC, read_bag, associate, apply_hand_eye,
)
from inchiscope_reconstruction.frame_selection import select_frames
from inchiscope_reconstruction.sanity_check import (
    sparse_sanity_check, scale_plausibility, filter_outlier_points, build_visualization_export,
)


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
    parser.add_argument('--clahe', action='store_true', help='apply CLAHE contrast enhancement before SIFT detection -- try this if the per-stage funnel below shows very few SIFT keypoints (e.g. <20/image), which is common on smooth, low-local-contrast content like mucosa')
    parser.add_argument('--clahe-clip-limit', type=float, default=4.0, help='CLAHE clip limit (default: 4.0, only used with --clahe)')
    parser.add_argument('--clahe-tile-size', type=int, default=8, help='CLAHE tile grid size, NxN (default: 8, only used with --clahe)')
    parser.add_argument('--expected-diameter-mm', type=float, required=True, help='expected scene scale (e.g. lumen diameter) in mm, used only as a coarse order-of-magnitude plausibility bound')
    parser.add_argument('--tolerance-factor', type=float, default=5.0, help='triangulated scale is flagged implausible outside [expected/factor, expected*factor] (default: 5.0)')
    parser.add_argument('--min-points-for-plausibility', type=int, default=20, help="don't trust the scale-plausibility check below this many triangulated points -- a handful of points can't give a meaningful extent estimate even if individually well-conditioned (default: 20)")
    parser.add_argument('--max-point-reproj-error-px', type=float, default=2.0, help='drop points whose reprojection error into either view exceeds this many px before computing scale plausibility -- these are either mismatches that coincidentally satisfied the epipolar line, or numerically unstable near-degenerate local triangulations, and can skew even a percentile-based extent estimate (default: 2.0)')
    parser.add_argument('--export-json', default=None, help='write triangulated points (grouped and labelled by which pair produced them, after the same outlier filtering as the report above) plus the full camera trajectory to this JSON file, for visual inspection -- colouring by pair reveals whether an inflated scale comes from pairs disagreeing with each other (pair-to-pair placement inconsistency) vs. one uniformly-too-large cloud (a real scale problem)')
    args = parser.parse_args()

    K, D, distortion_model, img_w, img_h = load_camera_intrinsics(args.camera_info)
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
        distortion_model=distortion_model,
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
        use_clahe=args.clahe,
        clahe_clip_limit=args.clahe_clip_limit,
        clahe_tile_size=args.clahe_tile_size,
    )

    diags = result['pair_diagnostics']
    baselines_mm = np.array([d['baseline_m'] for d in diags]) * 1000.0
    rotations_deg = np.array([d['rotation_deg'] for d in diags])
    print(f"Stage 3: attempted {result['pair_count']} pairs, {result['successful_pair_count']} "
          f"produced at least one triangulated point" + (' (CLAHE enabled)' if args.clahe else ''))
    print(f"  actual pose baseline between paired frames (--pair-stride={args.pair_stride} apart, "
          f"NOT the same as --min-baseline-m -- Stage 2 only guarantees that much between "
          f"CONSECUTIVE kept frames, real motion over --pair-stride frames can still be small): "
          f"translation min={baselines_mm.min():.2f}mm median={np.median(baselines_mm):.2f}mm "
          f"max={baselines_mm.max():.2f}mm; rotation min={rotations_deg.min():.2f} deg "
          f"median={np.median(rotations_deg):.2f} deg max={rotations_deg.max():.2f} deg")
    print('  per-stage funnel, summed across all attempted pairs:')
    print(f"    SIFT keypoints found (avg per image): "
          f"{sum(d['kp_a'] + d['kp_b'] for d in diags) / max(1, 2 * len(diags)):.0f}")
    print(f"    matches surviving ratio test:        {sum(d['ratio_matches'] for d in diags)}")
    print(f"    matches surviving epipolar check:    {sum(d['epipolar_matches'] for d in diags)}")
    print(f"    points surviving cheirality:         {sum(d['triangulated_points'] for d in diags)}")

    if result['point_count'] < args.min_points_for_plausibility:
        if result['point_count'] > 0:
            print(f"  {result['point_count']} point(s) triangulated -- too few to form a meaningful "
                  f"scale estimate (need >= --min-points-for-plausibility={args.min_points_for_plausibility}), "
                  f"even if individually well-conditioned. Treating this the same as zero for diagnosis "
                  f"purposes.", file=sys.stderr)
        avg_kp = sum(d['kp_a'] + d['kp_b'] for d in diags) / max(1, 2 * len(diags))
        avg_ratio = sum(d['ratio_matches'] for d in diags) / max(1, len(diags))
        avg_epi = sum(d['epipolar_matches'] for d in diags) / max(1, len(diags))
        if avg_kp < 20:
            if args.clahe:
                print('  DIAGNOSIS: SIFT is barely finding any keypoints even WITH --clahe enabled -- '
                      'this content may genuinely be too flat/low-texture for SIFT specifically '
                      '(contrast enhancement can only reveal real structure that\'s faintly present, '
                      'not invent structure from noise). Consider a detector better suited to low-'
                      'texture surfaces, or accept that sparse feature matching is not viable on this '
                      'footage and rely on Stage 4-5\'s dense stereo instead, which works on local '
                      'patch photo-consistency rather than needing distinctive sparse keypoints.', file=sys.stderr)
            else:
                print('  DIAGNOSIS: SIFT is barely finding any keypoints at all -- this content has too '
                      'little local contrast for SIFT specifically, independent of matching or pose '
                      'correctness (consistent with smooth, evenly-lit mucosa). Try --clahe (contrast-'
                      'limited adaptive histogram equalization) before drawing further conclusions -- '
                      'it can recover keypoints from real structure that\'s just poorly quantized, '
                      'though it cannot invent structure from truly flat/noise-floor content.', file=sys.stderr)
        elif avg_ratio < 8:
            print("  DIAGNOSIS: SIFT finds keypoints but the ratio test rejects nearly all matches -- "
                  "consistent with repetitive/self-similar texture (common on smooth mucosa) making "
                  "matches ambiguous, or --pair-stride being too wide for appearance to survive that "
                  "much viewpoint change. Try a smaller --pair-stride, or loosen --ratio (e.g. 0.85-0.9 "
                  "-- more false matches will get through, but the epipolar filter below should reject "
                  "genuinely wrong ones).", file=sys.stderr)
        elif avg_epi < 8:
            median_baseline_mm = float(np.median(baselines_mm))
            healthy_baseline_mm = max(2.0, 0.05 * args.expected_diameter_mm)
            if median_baseline_mm < healthy_baseline_mm:
                print(f'  DIAGNOSIS: matches are found, but nearly all fail the epipolar/cheirality '
                      f'checks -- AND the actual baseline between paired frames is small (median '
                      f'{median_baseline_mm:.2f}mm, vs a rough {healthy_baseline_mm:.1f}mm floor for '
                      f'well-conditioned triangulation at this scene scale). This looks like a near-'
                      f'degenerate baseline problem, not necessarily a bad hand-eye/pose stream: '
                      f'--pair-stride={args.pair_stride} isn\'t accumulating much real motion, likely '
                      f'because Stage 2\'s redundancy filter only guarantees a *minimum* gap between '
                      f'consecutive kept frames -- real motion over that many frames can still be tiny '
                      f'if the tip moved slowly for a stretch. Try a larger --pair-stride before '
                      f'suspecting calibration.', file=sys.stderr)
            else:
                print(f'  DIAGNOSIS: matches are found and the actual baseline between paired frames is '
                      f'reasonable (median {median_baseline_mm:.2f}mm), but nearly all matches still fail '
                      f'the KNOWN-pose epipolar check -- this is the signature of a bad hand-eye transform '
                      f'or corrupted pose stream (the geometry the matches are checked against is wrong), '
                      f'not a texture/baseline problem. Suspect the hand-eye calibration (still noisy/'
                      f'placeholder per project discussion) or the camera/Aurora timestamp mismatch '
                      f'during fast motion.', file=sys.stderr)
        else:
            print('  DIAGNOSIS: matches survive the epipolar check but all fail cheirality (behind one '
                  'or both cameras) -- suggests a sign/direction error somewhere in the pose or '
                  'projection convention rather than a data-quality problem.', file=sys.stderr)
        sys.exit(1)

    print(f"  reprojection error (px): mean={result['reproj_error_mean_px']:.2f} "
          f"median={result['reproj_error_median_px']:.2f} p90={result['reproj_error_p90_px']:.2f}")
    if result['reproj_error_median_px'] > 5.0:
        print('  WARNING: median reprojection error is high for triangulation against known, fixed '
              'poses (should generally be a few px at most) -- suggests the pose stream/hand-eye '
              'transform may be off even where matches survived the epipolar filter.', file=sys.stderr)

    filtered_points, kept_count, dropped_count = filter_outlier_points(
        result['points'], result['point_reproj_errors'], args.max_point_reproj_error_px,
    )
    print(f"  outlier filtering (reprojection error > {args.max_point_reproj_error_px}px): "
          f"kept {kept_count}, dropped {dropped_count}")
    if kept_count < args.min_points_for_plausibility:
        print(f'  Fewer than --min-points-for-plausibility={args.min_points_for_plausibility} points '
              f'survived outlier filtering -- not enough left to trust a scale estimate. Loosen '
              f'--max-point-reproj-error-px or gather more data before trusting this result.', file=sys.stderr)
        sys.exit(1)

    plausibility = scale_plausibility(filtered_points, args.expected_diameter_mm, args.tolerance_factor)
    lo, hi = plausibility['bounds_mm']
    print(f"  triangulated scene extent (mm, 5th-95th percentile per axis, outliers filtered): "
          f"{[round(v, 1) for v in plausibility['extent_mm']]}")
    print(f"  characteristic size: {plausibility['characteristic_size_mm']:.1f}mm "
          f"(plausible range given expected {args.expected_diameter_mm}mm +/- {args.tolerance_factor}x: "
          f"{lo:.1f}-{hi:.1f}mm)")

    if args.export_json:
        export_data = build_visualization_export(
            result, kept, args.expected_diameter_mm, args.max_point_reproj_error_px,
        )
        with open(args.export_json, 'w') as f:
            json.dump(export_data, f)
        total_exported = sum(len(p['points_mm']) for p in export_data['pairs'])
        print(f"  Exported {total_exported} points across {len(export_data['pairs'])} pairs, plus "
              f"{len(export_data['camera_trajectory_mm'])} camera trajectory positions, "
              f"to {args.export_json}")

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
