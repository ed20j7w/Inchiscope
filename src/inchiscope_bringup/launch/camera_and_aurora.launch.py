"""Phase 4 bench-test: camera feed + Aurora tracking together.

Brings up camera.launch.py and aurora.launch.py side by side -- for
watching the tip camera while EM tracking runs, ahead of recording a
synchronised rosbag of both (see record.launch.py / the top-level
README's Command reference). Each is a separate include so
show_viewer/use_rviz/rviz_config still work exactly as documented in
those two files.

Also publishes a static, zero-offset PLACEHOLDER transform from
aurora_sensor_0 to naneye_camera -- the camera and 6D EM sensor are
mounted together at the distal tip (see the manuscript), but their exact
rigid offset has never been measured. Without *some* transform, RViz's
Camera display can't resolve the image's naneye_camera frame at all (it
errors with "Frame [naneye_camera] does not exist"); zero offset at least
unblocks that, but treats the camera and sensor as co-located, which
they're physically not. Replace with the real measured offset once a
proper hand-eye/mechanical calibration is done.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('inchiscope_bringup'), 'launch'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch, 'camera.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch, 'aurora.launch.py')
            ),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='naneye_camera_placeholder_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'aurora_sensor_0',
                '--child-frame-id', 'naneye_camera',
            ],
        ),
    ])
