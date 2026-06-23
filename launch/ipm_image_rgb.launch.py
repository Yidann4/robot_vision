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
    
    ipm_node_yellow = Node(
        package='ipm_image_node',
        executable='ipm',
        output='screen',
        remappings=[
            #('camera_info', '/camera/camera_info'), ###FOR SIM
            ('camera_info', '/camera_info'), ###FOR REAL
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
    
    # bgr_to_rgb_yellow = Node(
    # launch_description.add_action(hsv_filter_node_yellow)
    #     package='robot_vision',
    #     executable='bgr_to_rgb_node',
    #     output='screen',
    #     remappings=[
    #         ('projected_point_cloud', '/ipm_cloud/yellow'),
    #     ],
    #     parameters=[{
    #         'colour': 'yellow'
    #     }]
    # )
    
    
    # bgr_to_rgb_blue = Node(
    #     package='robot_vision',
    #     executable='bgr_to_rgb_node',
    #     output='screen',
    #     remappings=[
    #         ('projected_point_cloud', 
    #          '/ipm_cloud/blue'),
    #     ],
    #     parameters=[{
    #         'colour': 'blue'
    #     }]
    # )
    

    launch_description = LaunchDescription()
    launch_description.add_action(hsv_filter_node)
    
    launch_description.add_action(ipm_node_yellow)
    # launch_description.add_action(bgr_to_rgb_yellow)
    
    launch_description.add_action(ipm_node_blue)
    # launch_description.add_action(bgr_to_rgb_blue)
    return launch_description
