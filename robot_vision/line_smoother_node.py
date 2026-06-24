# makes a smoother for blue and yellow

#!/usr/bin/env python3

from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class LineSmootherNode(Node):
	def __init__(self, colour):
		super().__init__(f'line_smoother_node_{colour}')

		self.declare_parameter('window_size', 5)
		self.declare_parameter('preserve_endpoints', True)

		self.path_sub = self.create_subscription(Path, f'/vision/path/{colour}', self.path_callback, 10)
		self.path_pub = self.create_publisher(Path, f'/vision/smoothed_path/{colour}', 10)

		self.get_logger().info(
			f'line_smoother_node started. input=/vision/path/{colour}, output=/vision/smoothed_path/{colour}'
		)

	def path_callback(self, msg: Path):
		if not msg.poses:
			self.path_pub.publish(msg)
			return

		smoothed_msg = Path()
		smoothed_msg.header = msg.header
		smoothed_msg.poses = self._smooth_poses(msg.poses)
		self.path_pub.publish(smoothed_msg)

	def _smooth_poses(self, poses: List[PoseStamped]) -> List[PoseStamped]:
		window_size = int(self.get_parameter('window_size').value)
		preserve_endpoints = bool(self.get_parameter('preserve_endpoints').value)

		n = len(poses)
		window_size = max(1, window_size)
		if window_size % 2 == 0:
			window_size += 1

		half_window = window_size // 2

		# Prefix sums make moving averages O(n) instead of O(n * window_size).
		prefix_x = [0.0]
		prefix_y = [0.0]
		prefix_z = [0.0]
		for pose in poses:
			prefix_x.append(prefix_x[-1] + float(pose.pose.position.x))
			prefix_y.append(prefix_y[-1] + float(pose.pose.position.y))
			prefix_z.append(prefix_z[-1] + float(pose.pose.position.z))

		smoothed: List[PoseStamped] = []
		for i, pose in enumerate(poses):
			if preserve_endpoints and (i == 0 or i == n - 1):
				smoothed_pose = PoseStamped()
				smoothed_pose.header = pose.header
				smoothed_pose.pose = pose.pose
				smoothed.append(smoothed_pose)
				continue

			left = max(0, i - half_window)
			right = min(n - 1, i + half_window)
			count = (right - left) + 1

			avg_x = (prefix_x[right + 1] - prefix_x[left]) / count
			avg_y = (prefix_y[right + 1] - prefix_y[left]) / count
			avg_z = (prefix_z[right + 1] - prefix_z[left]) / count

			smoothed_pose = PoseStamped()
			smoothed_pose.header = pose.header
			smoothed_pose.pose = pose.pose
			smoothed_pose.pose.position.x = avg_x
			smoothed_pose.pose.position.y = avg_y
			smoothed_pose.pose.position.z = avg_z
			smoothed.append(smoothed_pose)

		return smoothed


def main(args=None):
	rclpy.init(args=args)
	node_blue = LineSmootherNode('blue')
	node_yellow = LineSmootherNode('yellow')
	executor = MultiThreadedExecutor()
	executor.add_node(node_blue)
	executor.add_node(node_yellow)

	try:
		executor.spin()
	except KeyboardInterrupt:
		pass
	finally:
		executor.shutdown()
		node_blue.destroy_node()
		node_yellow.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
