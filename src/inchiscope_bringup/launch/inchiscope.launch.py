"""Full-system bring-up.

Starts every node in the stack against the shared params file. Nodes whose
control logic hasn't been implemented yet (pba_control, ab_control, control
-- see each node's docstring for phase/status) still start cleanly; their
action servers just abort goals with an explanatory message until that
logic lands. camera_node is functional as soon as a capture-card device is
present, and aurora_tracker_node is functional given the vendor SDK and
.rom files it needs -- see inchiscope_aurora/README.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('inchiscope_bringup'), 'config', 'params.yaml'
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
        Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[params_file],
        )
        for package, executable in nodes
    ])
