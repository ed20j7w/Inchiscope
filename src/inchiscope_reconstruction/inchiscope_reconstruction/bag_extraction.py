"""Stage 1 (extract & associate) of the reconstruction pipeline.

Reads a rosbag recorded via inchiscope_bringup's record.launch.py and
produces a time-ordered list of (timestamp, image, camera_pose_in_
reference_frame) tuples, ready for Stage 2 (frame_selection.py).

Uses /aurora/sensor_0/pose_relative_to_reference -- NOT the raw
/aurora/sensor_0/pose the original reconstruction workplan draft
referenced. The raw topic is in the field-generator's own frame, not
corrected to the fixed reference sensor, and isn't even recorded by
inchiscope_bringup/rosbag2/record_topics.yaml -- only the *_relative_to_
reference topic is, alongside /aurora/reference/pose for sanity-checking
the reference itself didn't move.

Split into a ROS-dependent I/O function (read_bag, needs rosbag2_py +
cv_bridge) and pure-Python/numpy functions (associate, apply_hand_eye)
that operate on plain (float, ndarray) tuples -- the pure functions can be
unit-tested with synthetic data with no ROS installation at all, which is
how they were validated here (see the module docstring of the test suite).
"""

import numpy as np

from inchiscope_reconstruction.calibration import quat_to_rotmat, to_T

IMAGE_TOPIC = '/camera/image_raw'
POSE_TOPIC = '/aurora/sensor_0/pose_relative_to_reference'


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(bag_path, image_topic=IMAGE_TOPIC, pose_topic=POSE_TOPIC):
    """Returns (images, poses):
        images = [(t_sec, bgr_image_ndarray), ...]
        poses  = [(t_sec, position_xyz_ndarray, orientation_xyzw_ndarray), ...]
    both sorted by time. storage_id is left for rosbag2_py to auto-detect
    from the bag's own metadata.yaml (don't assume sqlite3 vs mcap)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (image_topic, pose_topic):
        if topic not in topic_types:
            raise ValueError(f'bag {bag_path} does not contain topic {topic!r} '
                              f'(has: {sorted(topic_types)})')

    bridge = CvBridge()
    images = []
    poses = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == image_topic:
            msg = deserialize_message(data, Image)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            images.append((_stamp_to_sec(msg.header.stamp), img))
        elif topic == pose_topic:
            msg = deserialize_message(data, PoseStamped)
            p = msg.pose.position
            q = msg.pose.orientation
            poses.append((
                _stamp_to_sec(msg.header.stamp),
                np.array([p.x, p.y, p.z], dtype=np.float64),
                np.array([q.x, q.y, q.z, q.w], dtype=np.float64),
            ))
    images.sort(key=lambda entry: entry[0])
    poses.sort(key=lambda entry: entry[0])
    return images, poses


def associate(images, poses, max_pose_age_sec=0.1):
    """Nearest-in-time pose for each image. Images whose nearest pose is
    farther than max_pose_age_sec away are dropped rather than silently
    paired with a stale/future pose (mirrors calibrate_hand_eye.py
    capture's freshness check, applied here at extraction time instead of
    capture time since a bag's image/pose rates aren't independently
    controllable after the fact).

    Returns (associated, dropped_stale) where
    associated = [(t_img, image, position_xyz, orientation_xyzw, age_sec), ...]
    """
    if not poses:
        raise ValueError('no poses to associate against')
    pose_times = np.array([t for t, _, _ in poses])
    associated = []
    dropped = 0
    for t_img, img in images:
        idx = np.searchsorted(pose_times, t_img)
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(poses)]
        best = min(candidates, key=lambda i: abs(pose_times[i] - t_img))
        age = abs(pose_times[best] - t_img)
        if age > max_pose_age_sec:
            dropped += 1
            continue
        _, pos, quat = poses[best]
        associated.append((t_img, img, pos, quat, age))
    return associated, dropped


def apply_hand_eye(associated, R_cam2gripper, t_cam2gripper):
    """Converts each (Aurora sensor pose in reference frame) into (camera
    pose in reference frame): T_ref2cam = T_ref2gripper @ T_cam2gripper --
    the same composition validated in calibrate_hand_eye.py's synthetic
    test, applied here in the forward direction instead of being solved
    for.

    Returns [(t_img, image, T_ref2cam, age_sec), ...].
    """
    T_cg = to_T(R_cam2gripper, t_cam2gripper)
    out = []
    for t_img, img, pos, quat, age in associated:
        R_gripper = quat_to_rotmat(*quat)
        T_ref2gripper = to_T(R_gripper, pos)
        T_ref2cam = T_ref2gripper @ T_cg
        out.append((t_img, img, T_ref2cam, age))
    return out
