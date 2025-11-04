#!/usr/bin/env python3
"""
Launch file for Integrated 2D SLAM (Loop + IMU + Segmentation)

This is the most advanced SLAM configuration combining:
- Loop closure for global consistency
- IMU integration for high-speed motion
- Segmentation for dynamic object removal

Ideal for:
- Complex dynamic environments
- High-speed robot operation
- Large-scale mapping
- Maximum accuracy and robustness
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

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Integrated SLAM node
    slam_node = Node(
        package='gtsam_points_2d_slam',
        executable='slam_integrated_node',
        name='slam_integrated',
        output='screen',
        parameters=[
            os.path.join(pkg_slam, 'config', 'slam_integrated_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/imu', '/imu'),
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
