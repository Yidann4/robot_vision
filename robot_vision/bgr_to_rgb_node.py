import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class BGRtoRGBConverter(Node):
    def __init__(self):
        super().__init__('bgr_to_rgb_converter')
        self.sub = self.create_subscription(
            PointCloud2, '/projected_point_cloud', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/vision/projected_point_cloud_rgb', 10)

    def callback(self, msg):
        # Copy all fields
        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = msg.height
        new_msg.width = msg.width
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = msg.point_step
        new_msg.row_step = msg.row_step
        new_msg.is_dense = msg.is_dense

        # Get rgb offsets (offset 12 with point_step 16)
        rgb_offset = 12
        rgb_bytes = bytearray(msg.data)
        num_points = msg.width * msg.height

        # For each point, swap the first and third bytes (B <-> R)
        for i in range(num_points):
            pos = i * msg.point_step + rgb_offset
            b = rgb_bytes[pos]      # byte 0 = blue
            g = rgb_bytes[pos + 1]  # byte 1 = green
            r = rgb_bytes[pos + 2]  # byte 2 = red
            # Reorder to RGBA: R, G, B, alpha (alpha stays as is, usually 0)
            rgb_bytes[pos] = r
            rgb_bytes[pos + 1] = g
            rgb_bytes[pos + 2] = b
            # byte 3 (alpha) unchanged

        new_msg.data = bytes(rgb_bytes)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BGRtoRGBConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()