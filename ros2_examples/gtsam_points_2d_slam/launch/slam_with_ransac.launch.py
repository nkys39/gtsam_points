from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gtsam_points_2d_slam')
    config_file = os.path.join(pkg_share, 'config', 'slam_with_ransac_params.yaml')

    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # RANSAC SLAM node
    slam_with_ransac_node = Node(
        package='gtsam_points_2d_slam',
        executable='slam_with_ransac_node',
        name='slam_with_ransac',
        output='screen',
        parameters=[config_file]
    )

    # RViz node
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    return LaunchDescription([
        turtlebot3_gazebo,
        slam_with_ransac_node,
        rviz_node,
    ])
