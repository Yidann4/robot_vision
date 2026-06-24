from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    test_image_publisher = Node(
        package='robot_vision',
        executable='test_image_publisher',
        output='screen',
        parameters=[
            {'image_path': 'medium_black_arrow.png'},
        ],
    )

    easy_hsv_filter_node = Node(
        package='robot_vision',
        executable='easy_hsv_filter_node',
        output='screen',
        parameters=[
            {'input_topic': '/static_test/tuning_image'},
            {'output_topic': '/vision/hsv_mask/black'},
            {'h_min': 0},
            {'h_max': 179},
            {'s_min': 0},
            {'s_max': 255},
            {'v_min': 0},
            {'v_max': 150},
        ],
    )

    turning_challenge_classifier_node = Node(
        package='robot_vision',
        executable='turning_challenge_classifier_node',
        output='screen',
        parameters=[
            {'CONFIG_DEBUG': True},
        ],
    )

    return LaunchDescription([
        test_image_publisher,
        TimerAction(period=1.0, actions=[easy_hsv_filter_node]),
        TimerAction(period=2.0, actions=[turning_challenge_classifier_node]),
    ])
