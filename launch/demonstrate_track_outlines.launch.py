# Launches everything needed to create the /vision/line_points topics which are pointclouds of the racetrack lines
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('robot_vision')
    extract_lines_path = os.path.join(package_share, 'launch', 'extract_lines.launch.py')
    rviz_config_path = os.path.join(package_share, 'config', 'rviz_config.rviz')

    extract_lines_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(extract_lines_path)
    )
    
    point_cloud_binner = Node(
        package='robot_vision',
        executable='pointcloud_binner_node',
        output='screen',
    )

    lane_publisher_node = Node(
        package='robot_vision',
        executable='lane_publisher_node',
        output='screen',
    )
    
    lane_smoother_node = Node(
        package='robot_vision',
        executable='line_smoother_node',
        output='screen', 
    )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path] # Loads the custom config
        )

    launch_description = LaunchDescription()
    launch_description.add_action(extract_lines_launch)
    launch_description.add_action(point_cloud_binner)
    launch_description.add_action(lane_publisher_node)
    launch_description.add_action(rviz_node)
    return launch_description
