from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('robot_vision')

    hsv_parameters = os.path.join(package_share, 'config', 'ipm_tune_hsv_parameters.yaml')

    test_image_publisher_node = Node(
        package='robot_vision',
        executable='test_image_publisher',
        output='screen',
        parameters=[{
            'image_path': 'real_track.png'
        }],
    )

    easy_hsv_filter_node_yellow = Node(
        package='robot_vision',
        executable='easy_hsv_filter_node',
        output='screen',
        parameters=[{
            'input_topic': '/static_test/tuning_image',
            'output_topic': '/vision/hsv_mask/yellow',
            'h_min': 36,
            'h_max': 67,
            's_min': 78,
            's_max': 174,
            'v_min': 104,
            'v_max': 158,
        }],
    )
    
    easy_hsv_filter_node_blue = Node(
        package='robot_vision',
        executable='easy_hsv_filter_node',
        output='screen',
        parameters=[{
            'input_topic': '/static_test/tuning_image',
            'output_topic': '/vision/hsv_mask/blue',
            'h_min': 85,
            'h_max': 179,
            's_min': 152,
            's_max': 255,
            'v_min': 93,
            'v_max': 255,
        }],
    )


    
    rqt_image_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        output='screen',
        parameters=[{
            'image_transport': 'raw'
        }],
    )

    launch_description = LaunchDescription()
    launch_description.add_action(test_image_publisher_node)
    launch_description.add_action(easy_hsv_filter_node_yellow)
    launch_description.add_action(easy_hsv_filter_node_blue)
    launch_description.add_action(rqt_image_node)

    return launch_description