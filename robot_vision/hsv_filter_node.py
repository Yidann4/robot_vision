# publishes to both blue and yellow topics

#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class HSVFilterNode(Node):
	def __init__(self):
		super().__init__('hsv_filter_node')

		self.bridge = CvBridge()

		# Input topic
		self.declare_parameter('input_topic', '/static_test/tuning_image')

		# Output topics by colour
		self.declare_parameter('yellow_mask_topic', '/vision/hsv_mask/yellow')
		self.declare_parameter('yellow_filtered_topic', '/vision/hsv_filtered/yellow')
		self.declare_parameter('blue_mask_topic', '/vision/hsv_mask/blue')
		self.declare_parameter('blue_filtered_topic', '/vision/hsv_filtered/blue')

		# Yellow HSV bounds (OpenCV ranges: H 0-179, S 0-255, V 0-255)
		self.declare_parameter('yellow_h_min', 0)
		self.declare_parameter('yellow_h_max', 179)
		self.declare_parameter('yellow_s_min', 0)
		self.declare_parameter('yellow_s_max', 255)
		self.declare_parameter('yellow_v_min', 0)
		self.declare_parameter('yellow_v_max', 255)

		# Blue HSV bounds
		self.declare_parameter('blue_h_min', 90)
		self.declare_parameter('blue_h_max', 130)
		self.declare_parameter('blue_s_min', 60)
		self.declare_parameter('blue_s_max', 255)
		self.declare_parameter('blue_v_min', 40)
		self.declare_parameter('blue_v_max', 255)

		self.input_topic = self.get_parameter('input_topic').value
		self.yellow_mask_topic = self.get_parameter('yellow_mask_topic').value
		self.yellow_filtered_topic = self.get_parameter('yellow_filtered_topic').value
		self.blue_mask_topic = self.get_parameter('blue_mask_topic').value
		self.blue_filtered_topic = self.get_parameter('blue_filtered_topic').value

		self.sub = self.create_subscription(
			Image,
			self.input_topic,
			self.image_callback,
			10,
		)
		self.yellow_mask_pub = self.create_publisher(Image, self.yellow_mask_topic, 10)
		self.yellow_filtered_pub = self.create_publisher(Image, self.yellow_filtered_topic, 10)
		self.blue_mask_pub = self.create_publisher(Image, self.blue_mask_topic, 10)
		self.blue_filtered_pub = self.create_publisher(Image, self.blue_filtered_topic, 10)

		self.get_logger().info(
			f'HSV filter running. input={self.input_topic}, '
			f'yellow_mask={self.yellow_mask_topic}, yellow_filtered={self.yellow_filtered_topic}, '
			f'blue_mask={self.blue_mask_topic}, blue_filtered={self.blue_filtered_topic}'
		)

	def _get_hsv_bounds(self, colour: str):
		h_min = int(self.get_parameter(f'{colour}_h_min').value)
		h_max = int(self.get_parameter(f'{colour}_h_max').value)
		s_min = int(self.get_parameter(f'{colour}_s_min').value)
		s_max = int(self.get_parameter(f'{colour}_s_max').value)
		v_min = int(self.get_parameter(f'{colour}_v_min').value)
		v_max = int(self.get_parameter(f'{colour}_v_max').value)

		# Clamp to valid OpenCV HSV bounds.
		h_min = int(np.clip(h_min, 0, 179))
		h_max = int(np.clip(h_max, 0, 179))
		s_min = int(np.clip(s_min, 0, 255))
		s_max = int(np.clip(s_max, 0, 255))
		v_min = int(np.clip(v_min, 0, 255))
		v_max = int(np.clip(v_max, 0, 255))

		return (
			np.array([h_min, s_min, v_min], dtype=np.uint8),
			np.array([h_max, s_max, v_max], dtype=np.uint8),
		)

	def _publish_colour_outputs(self, mask, filtered, header, mask_pub, filtered_pub):
		mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
		mask_msg.header = header
		mask_pub.publish(mask_msg)

		filtered_msg = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
		filtered_msg.header = header
		filtered_pub.publish(filtered_msg)

	def image_callback(self, msg: Image):
		frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

		yellow_lower, yellow_upper = self._get_hsv_bounds('yellow')
		yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
		yellow_filtered = cv2.bitwise_and(frame_bgr, frame_bgr, mask=yellow_mask)
		self._publish_colour_outputs(
			yellow_mask,
			yellow_filtered,
			msg.header,
			self.yellow_mask_pub,
			self.yellow_filtered_pub,
		)

		blue_lower, blue_upper = self._get_hsv_bounds('blue')
		blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
		blue_filtered = cv2.bitwise_and(frame_bgr, frame_bgr, mask=blue_mask)
		self._publish_colour_outputs(
			blue_mask,
			blue_filtered,
			msg.header,
			self.blue_mask_pub,
			self.blue_filtered_pub,
		)


def main(args=None):
	rclpy.init(args=args)
	node = HSVFilterNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
