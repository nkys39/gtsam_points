#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_gtsam_points_2d_slam = get_package_share_directory('gtsam_points_2d_slam')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Config files
    slam_ct_gicp_params_file = os.path.join(
        pkg_gtsam_points_2d_slam,
        'config',
        'slam_with_ct_gicp_params.yaml'
    )

    # Launch TurtleBot3 Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # SLAM with CT-GICP node
    slam_ct_gicp_node = Node(
        package='gtsam_points_2d_slam',
        executable='slam_with_ct_gicp_node',
        name='slam_with_ct_gicp_node',
        output='screen',
        parameters=[
            slam_ct_gicp_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        gazebo_launch,
        slam_ct_gicp_node,
        rviz_node,
    ])
