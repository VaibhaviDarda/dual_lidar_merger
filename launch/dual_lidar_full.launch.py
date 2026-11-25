#!/usr/bin/env python3
# =============================================================================
#  dual_lidar_full.launch.py
#  Launches:
#    1. YDLidar driver node for LiDAR 1  (/scan)
#    2. YDLidar driver node for LiDAR 2  (/scan2)
#    3. dual_lidar_merger_node           (/scan_merged)
#    4. RViz2 for visualisation          (optional, default off)
#
#  Usage:
#    ros2 launch dual_lidar_merger dual_lidar_full.launch.py
#    ros2 launch dual_lidar_merger dual_lidar_full.launch.py rviz:=true
#    ros2 launch dual_lidar_merger dual_lidar_full.launch.py \
#        lidar1_port:=/dev/ttyUSB0 lidar2_port:=/dev/ttyUSB1
# =============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg = get_package_share_directory('dual_lidar_merger')

    # ── Declare launch arguments ─────────────────────────────────────────
    args = [
        DeclareLaunchArgument('rviz',          default_value='false',
                              description='Launch RViz2 for visualisation'),
        DeclareLaunchArgument('lidar1_port',   default_value='/dev/ttyUSB1',
                              description='Serial port for LiDAR 1 (front)'),
        DeclareLaunchArgument('lidar2_port',   default_value='/dev/ttyUSB0',
                              description='Serial port for LiDAR 2 (rear)'),
        DeclareLaunchArgument('overlap_mode',  default_value='nearest',
                              description='nearest | farthest | average'),
        DeclareLaunchArgument('merger_params', default_value=os.path.join(
                                  pkg, 'params', 'merger_params.yaml'),
                              description='Path to merger YAML'),
        DeclareLaunchArgument('lidar1_params', default_value=os.path.join(
                                  pkg, 'params', 'lidar1_params.yaml'),
                              description='Path to LiDAR 1 YAML'),
        DeclareLaunchArgument('lidar2_params', default_value=os.path.join(
                                  pkg, 'params', 'lidar2_params.yaml'),
                              description='Path to LiDAR 2 YAML'),
    ]

    # ── LiDAR 1 driver ───────────────────────────────────────────────────
    lidar1_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('lidar1_params'),
            {'port': LaunchConfiguration('lidar1_port')},
        ],
        remappings=[('scan', '/scan')],
    )

    # ── LiDAR 2 driver ───────────────────────────────────────────────────
    lidar2_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node_2',
        namespace='/',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('lidar2_params'),
            {'port': LaunchConfiguration('lidar2_port')},
        ],
        remappings=[('scan', '/scan2')],
    )

    # ── Merger node ───────────────────────────────────────────────────────
    merger_node = Node(
        package='dual_lidar_merger',
        executable='dual_lidar_merger_node',
        name='dual_lidar_merger',
        output='screen',
        parameters=[
            LaunchConfiguration('merger_params'),
            {'overlap_mode': LaunchConfiguration('overlap_mode')},
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(pkg, 'rviz', 'merged_scan.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(args + [
        lidar1_node,
        lidar2_node,
        merger_node,
        rviz_node,
    ])
