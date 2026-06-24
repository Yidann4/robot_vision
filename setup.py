from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    
    # specify which files to include in the build
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='bio.aidan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_image_publisher = robot_vision.test_image_publisher:main', #    ^executable     ^module path          ^function
            'hsv_filter_node = robot_vision.hsv_filter_node:main',
            'easy_hsv_filter_node = robot_vision.easy_hsv_filter_node:main',
            'hsv_tuner_node = robot_vision.hsv_tuner:main',
            'lane_publisher_node = robot_vision.lane_publisher:main',
            'pointcloud_binner_node = robot_vision.pointcloud_binner_node:main',
            'midpoint_drawer_node = robot_vision.midpoint_drawer_node:main',
            'turning_challenge_classifier_node = robot_vision.turning_challenge_classifier:main',
            'line_smoother_node = robot_vision.line_smoother_node:main',
        ],
    },
)
