"""Phase 4 bench-test: camera_node + an OpenCV viewer window.

Brings up the NanEye capture-card feed and a live cv2.imshow viewer,
without needing the rest of the stack running. See
camera_and_aurora.launch.py to bring this up alongside Aurora tracking,
and record.launch.py / the top-level README for recording a synchronised
rosbag of both afterwards.

Launch arguments:
    show_viewer (default true): start camera_viewer_node alongside
        camera_node. Set false on a headless machine, or if you'd rather
        just record without watching.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('inchiscope_bringup')
    params_file = os.path.join(bringup_share, 'config', 'params.yaml')

    show_viewer_arg = DeclareLaunchArgument(
        'show_viewer', default_value='true',
        description='Start camera_viewer_node (cv2.imshow window) alongside camera_node.',
    )

    return LaunchDescription([
        show_viewer_arg,
        Node(
            package='inchiscope_camera',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='inchiscope_camera',
            executable='camera_viewer_node',
            name='camera_viewer_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_viewer')),
        ),
    ])
