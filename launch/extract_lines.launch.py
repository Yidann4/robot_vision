from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('robot_vision')
    ipm_launch_path = os.path.join(package_share, 'launch', 'ipm_image_rgb.launch.py')

    ipm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ipm_launch_path)
    )

    line_filter_node = Node(
        package='robot_vision',
        executable='line_filter_node',
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(ipm_launch)
    launch_description.add_action(line_filter_node)
    return launch_description
