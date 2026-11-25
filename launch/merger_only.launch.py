#!/usr/bin/env python3
# =============================================================================
#  merger_only.launch.py
#  Launches ONLY the dual_lidar_merger_node.
#  Use this when your YDLidar driver nodes are already running
#  (e.g. via ydlidar_launch.py or your robot's main bringup launch).
#
#  Usage:
#    ros2 launch dual_lidar_merger merger_only.launch.py
#    ros2 launch dual_lidar_merger merger_only.launch.py overlap_mode:=average
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dual_lidar_merger')

    args = [
        DeclareLaunchArgument('merger_params', default_value=os.path.join(
                                  pkg, 'params', 'merger_params.yaml'),
                              description='Path to merger parameters YAML'),
        DeclareLaunchArgument('overlap_mode',  default_value='nearest',
                              description='nearest | farthest | average'),
        DeclareLaunchArgument('scan1_topic',   default_value='/scan',
                              description='Input topic from LiDAR 1'),
        DeclareLaunchArgument('scan2_topic',   default_value='/scan2',
                              description='Input topic from LiDAR 2'),
        DeclareLaunchArgument('output_topic',  default_value='/scan_merged',
                              description='Merged output topic'),
    ]

    merger_node = Node(
        package='dual_lidar_merger',
        executable='dual_lidar_merger_node',
        name='dual_lidar_merger',
        output='screen',
        parameters=[
            LaunchConfiguration('merger_params'),
            # CLI overrides beat YAML values
            {'overlap_mode':  LaunchConfiguration('overlap_mode')},
            {'scan1_topic':   LaunchConfiguration('scan1_topic')},
            {'scan2_topic':   LaunchConfiguration('scan2_topic')},
            {'output_topic':  LaunchConfiguration('output_topic')},
        ],
    )

    return LaunchDescription(args + [merger_node])
