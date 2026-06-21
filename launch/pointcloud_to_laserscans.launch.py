from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_vision'),
        'config',
        'point_cloud_to_laserscan_custom.yaml',
    )

    # yellow_scan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='laserscan_node_yellow',
    #     output='screen',
    #     parameters=[params_file],
    #     remappings=[
    #         ('cloud_in', '/vision/line_points/yellow'),
    #         ('scan', '/vision/scan/yellow'),
    #     ],
    # )

    # blue_scan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='laserscan_node_blue',
    #     output='screen',
    #     parameters=[params_file],
    #     remappings=[
    #         ('cloud_in', '/vision/line_points/blue'),
    #         ('scan', '/vision/scan/blue'),
    #     ],
    # )
    
    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='laserscan_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cloud_in', '/vision/line_points'),
            ('scan', '/vision/scan'),
        ],
    )

    return LaunchDescription([
        scan_node,
    ])