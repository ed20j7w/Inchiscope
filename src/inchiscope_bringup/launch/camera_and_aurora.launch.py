"""Phase 4 bench-test: camera feed + Aurora tracking together.

Brings up camera.launch.py and aurora.launch.py side by side -- for
watching the tip camera while EM tracking runs, ahead of recording a
synchronised rosbag of both (see record.launch.py / the top-level
README's Command reference). Each is a separate include so
show_viewer/use_rviz/rviz_config still work exactly as documented in
those two files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    ])
