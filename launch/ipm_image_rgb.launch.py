from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ipm_node_yellow = Node(
        package='ipm_image_node',
        executable='ipm',
        output='screen',
        remappings=[
            ('camera_info', '/camera/camera_info'),
            ('input', '/vision/hsv_mask/yellow'),
            ('projected_point_cloud', '/ipm_cloud/yellow'),
        ],
        parameters=[{
            'type': 'mask',
            'output_frame': 'base_footprint',
        }],
    )
    
    ipm_node_blue = Node(
        package='ipm_image_node',
        executable='ipm',
        output='screen',
        remappings=[
            ('camera_info', '/camera/camera_info'),
            ('input', '/vision/hsv_mask/blue'),
            ('projected_point_cloud', '/ipm_cloud/blue'),
        ],
        parameters=[{
            'type': 'mask',
            'output_frame': 'base_footprint',
        }],
    )


    launch_description = LaunchDescription()
    
    launch_description.add_action(ipm_node_yellow)
    
    launch_description.add_action(ipm_node_blue)
    return launch_description
