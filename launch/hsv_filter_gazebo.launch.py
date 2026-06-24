from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hsv_parameters = os.path.join(
        get_package_share_directory('robot_vision'),
        'config',
        'hsv_parameters.yaml'
    )

    hsv_filter_node = Node(
        package='robot_vision',
        executable='hsv_filter_node',
        output='screen',
        parameters=[hsv_parameters],
    )
    

    launch_description = LaunchDescription()
    launch_description.add_action(hsv_filter_node)
    
    return launch_description
