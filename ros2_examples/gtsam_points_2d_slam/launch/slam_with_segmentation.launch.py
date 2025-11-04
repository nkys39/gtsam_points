#!/usr/bin/env python3
"""
Launch file for 2D SLAM with Region Growing Segmentation

This launch file starts:
- TurtleBot3 Gazebo simulation
- SLAM node with segmentation for dynamic object removal
- RViz2 for visualization

Segmentation Features:
- Region Growing segmentation of laser scans
- Dynamic object detection and filtering
- Static-only SLAM for robust mapping in dynamic environments
- Semantic labeling visualization (green=static, red=dynamic)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_slam = get_package_share_directory('gtsam_points_2d_slam')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(
        pkg_gazebo, 'worlds', 'turtlebot3_world.world'))

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # SLAM node with segmentation
    slam_node = Node(
        package='gtsam_points_2d_slam',
        executable='slam_with_segmentation_node',
        name='slam_with_segmentation',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'slam_with_segmentation_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    # RViz2
    rviz_config = os.path.join(pkg_slam, 'config', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'),

        gazebo_launch,
        slam_node,
        rviz_node,
    ])
