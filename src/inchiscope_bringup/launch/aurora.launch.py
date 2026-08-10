"""Phase 4 bench-test: aurora_tracker_node, optionally with RViz.

Brings up just the Aurora EM tracker node against the params file, plus
RViz for visualising the reference/sensor poses, without needing the rest
of the stack (firmware, camera, etc.) running.

Launch arguments:
    use_rviz (default true): whether to also start RViz.
    rviz_config (default: rviz/aurora.rviz in this package's share dir):
        path to an RViz config file. It's fine if this doesn't exist yet --
        RViz falls back to its default empty layout and logs a warning; see
        the top-level README for how to set up displays and save one here.
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
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'aurora.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start RViz alongside aurora_tracker_node.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='RViz config file to open (need not exist yet).',
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        Node(
            package='inchiscope_aurora',
            executable='aurora_tracker_node',
            name='aurora_tracker_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
