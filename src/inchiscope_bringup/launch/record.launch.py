"""Phase 5: rosbag2 recording for offline 3D reconstruction.

Records exactly the topics listed in rosbag2/record_topics.yaml -- image
stream, EM tracker pose, and high-level state -- and deliberately leaves out
the high-rate low-level firmware telemetry so the bag stays a reasonable
size. See inchiscope_ros2_architecture.md section 6, phase 5.

Launch arguments:
    bag_name (default: inchiscope_bag_<timestamp>): output bag directory
        name, created relative to wherever `ros2 launch` was run from.
        Defaults to a timestamp so repeated recordings don't collide --
        `ros2 bag record` refuses to overwrite an existing directory.
        Pass an explicit name (e.g. bag_name:=trial_1) for something more
        descriptive.
"""

import datetime
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    topics_file = os.path.join(
        get_package_share_directory('inchiscope_bringup'), 'rosbag2', 'record_topics.yaml'
    )
    with open(topics_file) as f:
        topics = yaml.safe_load(f)['topics']

    default_bag_name = 'inchiscope_bag_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_name_arg = DeclareLaunchArgument(
        'bag_name', default_value=default_bag_name,
        description=(
            'Output bag directory name (relative to cwd). Defaults to a '
            'timestamp so repeated recordings do not collide -- ros2 bag '
            'record refuses to overwrite an existing directory.'
        ),
    )

    return LaunchDescription([
        bag_name_arg,
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_name'), *topics],
            output='screen',
        ),
    ])
