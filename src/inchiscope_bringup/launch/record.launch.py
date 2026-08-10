"""Phase 5: rosbag2 recording for offline 3D reconstruction.

Records exactly the topics listed in rosbag2/record_topics.yaml -- image
stream, EM tracker pose, and high-level state -- and deliberately leaves out
the high-rate low-level firmware telemetry so the bag stays a reasonable
size. See inchiscope_ros2_architecture.md section 6, phase 5.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    topics_file = os.path.join(
        get_package_share_directory('inchiscope_bringup'), 'rosbag2', 'record_topics.yaml'
    )
    with open(topics_file) as f:
        topics = yaml.safe_load(f)['topics']

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'inchiscope_bag', *topics],
            output='screen',
        ),
    ])
