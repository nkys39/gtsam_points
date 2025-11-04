from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gtsam_points_2d_slam')

    # Declare launch arguments
    use_vgicp_arg = DeclareLaunchArgument(
        'use_vgicp',
        default_value='true',
        description='Use VGICP instead of GICP'
    )

    # Offline map optimizer node
    offline_map_optimizer_node = Node(
        package='gtsam_points_2d_slam',
        executable='offline_map_optimizer',
        name='offline_map_optimizer',
        output='screen',
        parameters=[{
            'map_frame': 'map',
            'use_vgicp': LaunchConfiguration('use_vgicp'),
            'voxel_resolution': 0.1,
            'max_correspondence_distance': 1.0,
            'loop_closure_threshold': 0.8,
        }]
    )

    # RViz node for visualization
    rviz_config = os.path.join(pkg_share, 'rviz', 'offline_optimizer.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen'
    )

    return LaunchDescription([
        use_vgicp_arg,
        offline_map_optimizer_node,
        rviz_node,
    ])
