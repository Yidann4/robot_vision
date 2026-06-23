import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import message_filters
import numpy as np

class CenterlineGenerator(Node):
    def __init__(self):
        super().__init__('centerline_generator')

        self.extension_length = 0.4  # Meters to extend beyond the last point

        # 1. Setup synchronized subscribers for the two boundaries
        self.blue_sub = message_filters.Subscriber(self, Path, '/vision/smoothed_path/blue')
        self.yellow_sub = message_filters.Subscriber(self, Path, '/vision/smoothed_path/yellow')

        # Adjust slop (seconds) based on your pipeline's latency
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.blue_sub, self.yellow_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.path_callback)

        # 2. Publisher for the calculated mid-path
        self.centerline_pub = self.create_publisher(Path, '/map/centerline_path', 10)
        self.get_logger().info('Centerline path generator initialized.')

    def path_callback(self, blue_path, yellow_path):
        # Edge case: Ensure neither path is empty
        if not blue_path.poses or not yellow_path.poses:
            return

        # Convert yellow path positions to a numpy array for fast distance math
        yellow_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in yellow_path.poses])

        centerline_msg = Path()
        centerline_msg.header = blue_path.header  # Maintain frame and synchronized timestamp

        # Loop through each point in the blue path and find its closest yellow partner
        for b_pose in blue_path.poses:
            b_x = b_pose.pose.position.x
            b_y = b_pose.pose.position.y

            # Calculate Euclidean distances to all yellow points
            distances = np.linalg.norm(yellow_pts - np.array([b_x, b_y]), axis=1)
            closest_idx = np.argmin(distances)
            
            # Extract the closest yellow point coordinates
            y_x = yellow_pts[closest_idx][0]
            y_y = yellow_pts[closest_idx][1]

            # Compute the midpoint
            mid_x = (b_x + y_x) / 2.0
            mid_y = (b_y + y_y) / 2.0

            # Append to the new path message
            pose_stamped = PoseStamped()
            pose_stamped.header = centerline_msg.header
            pose_stamped.pose.position.x = mid_x
            pose_stamped.pose.position.y = mid_y
            pose_stamped.pose.position.z = b_pose.pose.position.z  # Match elevation
            
            centerline_msg.poses.append(pose_stamped)

        # Append one forward extrapolated point 0.2 m beyond the last midpoint.
        if len(centerline_msg.poses) >= 2:
            prev_pose = centerline_msg.poses[-2].pose.position
            last_pose = centerline_msg.poses[-1].pose.position

            dx = last_pose.x - prev_pose.x
            dy = last_pose.y - prev_pose.y
            seg_len = float(np.hypot(dx, dy))

            if seg_len > 1e-6:
                extension = self.extension_length
                ex = (dx / seg_len) * extension
                ey = (dy / seg_len) * extension

                end_pose = PoseStamped()
                end_pose.header = centerline_msg.header
                end_pose.pose.position.x = last_pose.x + ex
                end_pose.pose.position.y = last_pose.y + ey
                end_pose.pose.position.z = last_pose.z
                end_pose.pose.orientation.w = 1.0
                centerline_msg.poses.append(end_pose)

        # Publish the final centerline
        self.centerline_pub.publish(centerline_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CenterlineGenerator()
    rclpy.spin(node)
    rclpy.shutdown()