# Run with:
# ros2 run robot_vision easy_hsv_filter_node --ros-args -p input_topic:=/static_test/tuning_image -p output_topic:=/vision/hsv_mask -p h_min:=0 -p h_max:=179 -p s_min:=0 -p s_max:=255 -p v_min:=0 -p v_max:=255

#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class EasyHSVFilterNode(Node):
	def __init__(self):
		super().__init__('easy_hsv_filter_node')

		self.bridge = CvBridge()

		self.declare_parameter('input_topic', '/camera/image_raw')
		self.declare_parameter('output_topic', '/vision/hsv_mask')

		# OpenCV HSV ranges: H 0-179, S 0-255, V 0-255
		self.declare_parameter('h_min', 0)
		self.declare_parameter('h_max', 179)
		self.declare_parameter('s_min', 0)
		self.declare_parameter('s_max', 255)
		self.declare_parameter('v_min', 0)
		self.declare_parameter('v_max', 255)

		# Morphology controls
		self.declare_parameter('morph_kernel_size', 3)
		self.declare_parameter('erode_iterations', 1)
		self.declare_parameter('dilate_iterations', 1)

		self.input_topic = str(self.get_parameter('input_topic').value)
		self.output_topic = str(self.get_parameter('output_topic').value)

		self.subscription = self.create_subscription(
			Image,
			self.input_topic,
			self.image_callback,
			10,
		)
		self.mask_publisher = self.create_publisher(Image, self.output_topic, 10)

		self.get_logger().info(
			f'easy_hsv_filter_node started. input={self.input_topic}, output={self.output_topic}'
		)

	def _get_hsv_bounds(self):
		h_min = int(np.clip(int(self.get_parameter('h_min').value), 0, 179))
		h_max = int(np.clip(int(self.get_parameter('h_max').value), 0, 179))
		s_min = int(np.clip(int(self.get_parameter('s_min').value), 0, 255))
		s_max = int(np.clip(int(self.get_parameter('s_max').value), 0, 255))
		v_min = int(np.clip(int(self.get_parameter('v_min').value), 0, 255))
		v_max = int(np.clip(int(self.get_parameter('v_max').value), 0, 255))

		lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
		upper = np.array([h_max, s_max, v_max], dtype=np.uint8)
		return lower, upper

	def image_callback(self, msg: Image):
		frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

		lower, upper = self._get_hsv_bounds()
		mask = cv2.inRange(frame_hsv, lower, upper)

		kernel_size = int(self.get_parameter('morph_kernel_size').value)
		erode_iterations = int(self.get_parameter('erode_iterations').value)
		dilate_iterations = int(self.get_parameter('dilate_iterations').value)

		kernel_size = max(1, kernel_size)
		if kernel_size % 2 == 0:
			kernel_size += 1

		erode_iterations = max(0, erode_iterations)
		dilate_iterations = max(0, dilate_iterations)

		kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
		if erode_iterations > 0:
			mask = cv2.erode(mask, kernel, iterations=erode_iterations)
		if dilate_iterations > 0:
			mask = cv2.dilate(mask, kernel, iterations=dilate_iterations)

		mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
		mask_msg.header = msg.header
		self.mask_publisher.publish(mask_msg)


def main(args=None):
	rclpy.init(args=args)
	node = EasyHSVFilterNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
