"""Phase 1 bring-up: firmware + serial bridge only.

Brings up just the serial_bridge_node against the params file, so the
low-level loop (PISTON/VALVE/PING <-> TEL/ACK/ERR) can be bench-tested with
a manual CLI (`ros2 topic pub ...` or a scratch script) before any of the
higher-level control nodes exist. See inchiscope_ros2_architecture.md
section 6, phase 1.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('inchiscope_bringup'), 'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='inchiscope_serial_bridge',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
