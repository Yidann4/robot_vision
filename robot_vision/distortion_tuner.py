#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class DistortionTuner(Node):

    def __init__(self):
        super().__init__('distortion_tuner')

        self.bridge = CvBridge()

        # Camera matrix from your calibration
        self.K = np.array([
            [216.51078, -0.92533, 319.68753],
            [0.0,       218.64979, 238.17170],
            [0.0,       0.0,       1.0]
        ], dtype=np.float64)

        self.base_D = np.array([
            0.248201,
           -1.308089,
            1.614096,
           -0.372616
        ], dtype=np.float64)

        self.image_size = (640, 480)

        # Runtime tunable parameters
        self.declare_parameter('distortion_scale', 1.0)
        self.declare_parameter('balance', 0.0)

        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            CompressedImage,
            '/image_undistorted/compressed',
            10
        )

        self._rebuild_maps()

        self.get_logger().info('Distortion tuner started')

    def _rebuild_maps(self):

        scale = self.get_parameter(
            'distortion_scale').get_parameter_value().double_value

        balance = self.get_parameter(
            'balance').get_parameter_value().double_value

        D = self.base_D * scale

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K,
            D,
            self.image_size,
            np.eye(3),
            balance=balance
        )

        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K,
            D,
            np.eye(3),
            new_K,
            self.image_size,
            cv2.CV_16SC2
        )

        self.last_scale = scale
        self.last_balance = balance

    def image_callback(self, msg):

        scale = self.get_parameter(
            'distortion_scale').get_parameter_value().double_value

        balance = self.get_parameter(
            'balance').get_parameter_value().double_value

        if (
            scale != self.last_scale or
            balance != self.last_balance
        ):
            self._rebuild_maps()

        frame = self.bridge.compressed_imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        corrected = cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR
        )

        out = self.bridge.cv2_to_compressed_imgmsg(corrected)
        out.header = msg.header

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = DistortionTuner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()