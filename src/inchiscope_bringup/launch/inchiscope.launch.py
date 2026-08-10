"""Full-system bring-up.

Starts every node in the stack against the shared params file. Nodes whose
control logic hasn't been implemented yet (pba_control, ab_control, control
-- see each node's docstring for phase/status) still start cleanly; their
action servers just abort goals with an explanatory message until that
logic lands. camera_node is functional as soon as a capture-card device is
present, and aurora_tracker_node is functional given the vendor SDK and
.rom files it needs -- see inchiscope_aurora/README.md.

Launch arguments:
    use_rviz (default false): also start RViz. Off by default here since a
        full-stack launch shouldn't always pop a GUI; see aurora.launch.py
        for a bench-test launch with it on by default.
    rviz_config (default: rviz/aurora.rviz in this package's share dir).
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
        'use_rviz', default_value='false',
        description='Start RViz alongside the rest of the stack.',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=default_rviz_config,
        description='RViz config file to open (need not exist yet).',
    )

    nodes = [
        ('inchiscope_serial_bridge', 'serial_bridge_node'),
        ('inchiscope_pba_control', 'pba_control_node'),
        ('inchiscope_ab_control', 'ab_control_node'),
        ('inchiscope_control', 'inchiscope_control_node'),
        ('inchiscope_aurora', 'aurora_tracker_node'),
        ('inchiscope_camera', 'camera_node'),
    ]

    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        *[
            Node(
                package=package,
                executable=executable,
                name=executable,
                output='screen',
                parameters=[params_file],
            )
            for package, executable in nodes
        ],
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
