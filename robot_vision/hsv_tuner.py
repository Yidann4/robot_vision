#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class HSVCalibratorNode(Node):
    def __init__(self):
        super().__init__('hsv_calibrator')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        self.latest_img = None

        # Create windows
        cv2.namedWindow('mask')
        cv2.namedWindow('original')
        cv2.createTrackbar('H_min', 'mask', 0, 179, lambda x: None)
        cv2.createTrackbar('H_max', 'mask', 179, 179, lambda x: None)
        cv2.createTrackbar('S_min', 'mask', 0, 255, lambda x: None)
        cv2.createTrackbar('S_max', 'mask', 255, 255, lambda x: None)
        cv2.createTrackbar('V_min', 'mask', 0, 255, lambda x: None)
        cv2.createTrackbar('V_max', 'mask', 255, 255, lambda x: None)

    def image_callback(self, msg):
        try:
            self.latest_img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to decode compressed image: {exc}')

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.latest_img is None:
                continue
            h_min = cv2.getTrackbarPos('H_min', 'mask')
            h_max = cv2.getTrackbarPos('H_max', 'mask')
            s_min = cv2.getTrackbarPos('S_min', 'mask')
            s_max = cv2.getTrackbarPos('S_max', 'mask')
            v_min = cv2.getTrackbarPos('V_min', 'mask')
            v_max = cv2.getTrackbarPos('V_max', 'mask')

            hsv = cv2.cvtColor(self.latest_img, cv2.COLOR_BGR2HSV)
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)

            cv2.imshow('original', self.latest_img)
            cv2.imshow('mask', mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibratorNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()