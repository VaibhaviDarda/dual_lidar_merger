#!/usr/bin/env python3
# =============================================================================
#  view_merged.launch.py
#  Opens RViz2 pre-configured to display /scan, /scan2, and /scan_merged.
#  Run this in a separate terminal while your robot is already publishing scans.
#
#  Usage:
#    ros2 launch dual_lidar_merger view_merged.launch.py
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dual_lidar_merger')
    rviz_cfg = os.path.join(pkg, 'rviz', 'merged_scan.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([rviz_node])
