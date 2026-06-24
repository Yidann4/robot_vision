# Launches everything needed to create the /vision/line_points topics which are pointclouds of the racetrack lines
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('robot_vision')
    demonstrate_track_outlines_path = os.path.join(package_share, 'launch', 'demonstrate_track_outlines.launch.py')
    rviz_config_path = os.path.join(package_share, 'config', 'rviz_config.rviz')

    demonstrate_track_outlines = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demonstrate_track_outlines_path)
    )
    
    mid_point_draw = Node(
        package='robot_vision',
        executable='midpoint_drawer_node',
        output='screen',
    )


    launch_description = LaunchDescription()
    launch_description.add_action(demonstrate_track_outlines)
    launch_description.add_action(mid_point_draw)
    return launch_description
