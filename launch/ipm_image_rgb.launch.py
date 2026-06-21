from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ipm_node = Node(
        package='ipm_image_node',
        executable='ipm',
        output='screen',
        remappings=[
            ('camera_info', '/camera/camera_info'),
            ('input', '/camera/image_raw'),
        ],
        parameters=[{
            'type': 'rgb_image',
            'output_frame': 'base_footprint',
            'ipm_scale': 0.25
        }],
    )
    
    bgr_to_rgb = Node(
        package='robot_vision',
        executable='bgr_to_rgb_node',
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(ipm_node)
    launch_description.add_action(bgr_to_rgb)
    return launch_description
