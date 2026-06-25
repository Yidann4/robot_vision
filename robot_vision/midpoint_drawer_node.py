import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import message_filters
import math
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
        
        self.turning_challenge_sub = self.create_subscription(String, '/vision/turning_challenge/direction', self.turning_challenge_callback, 10)
        self.turning_challenge = None
        
        self.declare_parameter('CONFIG_DEBUG', False)
        self.debug = self.get_parameter('CONFIG_DEBUG').value
        
        
        self.declare_parameter('look_ahead_distance', 0.6)
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').value
        self.declare_parameter('min_path_length', 0.15)
        self.min_path_length = self.get_parameter('min_path_length').value
        self.pure_pursuit_point_pub = self.create_publisher(PoseStamped, '/map/pure_pursuit_point', 10)

    def path_callback(self, blue_path, yellow_path):
        # Edge case: Ensure neither path is empty
        if not blue_path.poses or not yellow_path.poses:
            return

        if self.turning_challenge == None:
            centerline_msg = self.do_regular_midpoint(blue_path, yellow_path)
        elif self.turning_challenge == 'left':
            centerline_msg = self.do_left_turn_midpoint(yellow_path)
        elif self.turning_challenge == 'right':
            centerline_msg = self.do_right_turn_midpoint(blue_path)
            
        if self.debug:
            self.get_logger().info(f'Doing midpoint of type {self.turning_challenge}')
            
        
        # Publish the final centerline
        self.centerline_pub.publish(centerline_msg)
        
        self.maybe_publish_pure_pursuit_point(centerline_msg, blue_path, yellow_path)
        
    def turning_challenge_callback(self, msg):
        self.turning_challenge = msg.data
        self.get_logger().info(f'Turning challenge set to: {self.turning_challenge}')
    
    def do_regular_midpoint(self, blue_path, yellow_path):
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
        return centerline_msg
    
    def do_right_turn_midpoint(self, blue_path):
        centerline_msg = Path()
        centerline_msg.header = blue_path.header

        shift_distance = 0.4  # 30 cm to the left of blue boundary
        n = len(blue_path.poses)

        for i, b_pose in enumerate(blue_path.poses):
            p = b_pose.pose.position

            # Estimate local forward tangent from neighboring points.
            if n == 1:
                dx, dy = 1.0, 0.0
            elif i == 0:
                p_next = blue_path.poses[i + 1].pose.position
                dx, dy = p_next.x - p.x, p_next.y - p.y
            elif i == n - 1:
                p_prev = blue_path.poses[i - 1].pose.position
                dx, dy = p.x - p_prev.x, p.y - p_prev.y
            else:
                p_prev = blue_path.poses[i - 1].pose.position
                p_next = blue_path.poses[i + 1].pose.position
                dx, dy = p_next.x - p_prev.x, p_next.y - p_prev.y

            seg_len = float(np.hypot(dx, dy))
            if seg_len < 1e-6:
                rx, ry = 0.0, 0.0
            else:
                # Left-hand normal for a heading vector [dx, dy]
                rx, ry = -dy / seg_len, dx / seg_len

            shifted_pose = PoseStamped()
            shifted_pose.header = centerline_msg.header
            shifted_pose.pose.position.x = p.x + shift_distance * rx
            shifted_pose.pose.position.y = p.y + shift_distance * ry
            shifted_pose.pose.position.z = p.z
            shifted_pose.pose.orientation = b_pose.pose.orientation
            centerline_msg.poses.append(shifted_pose)

        return centerline_msg
    
    def do_left_turn_midpoint(self, yellow_path):
        centerline_msg = Path()
        centerline_msg.header = yellow_path.header

        shift_distance = 0.4  # 30 cm to the right of yellow boundary
        n = len(yellow_path.poses)

        for i, y_pose in enumerate(yellow_path.poses):
            p = y_pose.pose.position

            # Estimate local forward tangent from neighboring points.
            if n == 1:
                dx, dy = 1.0, 0.0
            elif i == 0:
                p_next = yellow_path.poses[i + 1].pose.position
                dx, dy = p_next.x - p.x, p_next.y - p.y
            elif i == n - 1:
                p_prev = yellow_path.poses[i - 1].pose.position
                dx, dy = p.x - p_prev.x, p.y - p_prev.y
            else:
                p_prev = yellow_path.poses[i - 1].pose.position
                p_next = yellow_path.poses[i + 1].pose.position
                dx, dy = p_next.x - p_prev.x, p_next.y - p_prev.y

            seg_len = float(np.hypot(dx, dy))
            if seg_len < 1e-6:
                rx, ry = 0.0, 0.0
            else:
                # Right-hand normal for a heading vector [dx, dy]
                rx, ry = dy / seg_len, -dx / seg_len

            shifted_pose = PoseStamped()
            shifted_pose.header = centerline_msg.header
            shifted_pose.pose.position.x = p.x + shift_distance * rx
            shifted_pose.pose.position.y = p.y + shift_distance * ry
            shifted_pose.pose.position.z = p.z
            shifted_pose.pose.orientation = y_pose.pose.orientation
            centerline_msg.poses.append(shifted_pose)

        return centerline_msg
    
    def maybe_publish_pure_pursuit_point(self, centerline_msg, blue_path, yellow_path):
        if not centerline_msg.poses:
            return

        if self.should_filter_pure_pursuit_point(blue_path, yellow_path):
            return

        
        target_distance = self.look_ahead_distance       
        accumulated_distance = 0.0
        pure_pursuit_point = None
        
        # walk every segment length
        for i in range(len(centerline_msg.poses) - 1):
            p0 = centerline_msg.poses[i].pose.position
            p1 = centerline_msg.poses[i + 1].pose.position

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            segment_len = math.sqrt(dx*dx + dy*dy + dz*dz)

            # once point is past look ahead interpolate to find exact point at look ahead distance
            if accumulated_distance + segment_len >= target_distance:
                if segment_len < 1e-9:
                    accumulated_distance += segment_len
                    continue

                # The target point lies within this segment
                remainder = target_distance - accumulated_distance
                t = remainder / segment_len          # 0.0 → 1.0 along this segment

                result = PoseStamped()
                result.header = centerline_msg.poses[i].header       # inherits frame_id and stamp
                result.pose.position.x = p0.x + t * dx
                result.pose.position.y = p0.y + t * dy
                result.pose.position.z = p0.z + t * dz

                # Interpolate orientation (slerp would be ideal, simple lerp shown here)
                result.pose.orientation = centerline_msg.poses[i].pose.orientation

                pure_pursuit_point = result
                break
            

            accumulated_distance += segment_len
        
        if pure_pursuit_point is not None:
            self.pure_pursuit_point_pub.publish(pure_pursuit_point)

    # check if point should be removed (not sent)
    def should_filter_pure_pursuit_point(self, blue_path, yellow_path):
        blue_length = self.compute_path_length(blue_path)
        yellow_length = self.compute_path_length(yellow_path)
        if blue_length < self.min_path_length or yellow_length < self.min_path_length:
            if self.debug:
                self.get_logger().info(
                    f'Skipping pure pursuit publish due to short boundary path length '
                    f'(blue={blue_length:.3f} m, yellow={yellow_length:.3f} m, '
                    f'min={self.min_path_length:.3f} m)'
                )
            return True

        blue_is_right, right_ratio = self.is_blue_mostly_right_of_yellow(blue_path, yellow_path)
        if not blue_is_right:
            if self.debug:
                self.get_logger().info(
                    f'Skipping pure pursuit publish because blue path is not mostly right of yellow '
                    f'(blue_y < yellow_y ratio={right_ratio:.3f}, required>0.500)'
                )
            return True

        return False

    def is_blue_mostly_right_of_yellow(self, blue_path, yellow_path):
        if not blue_path.poses or not yellow_path.poses:
            return False, 0.0

        yellow_pts = np.array([[p.pose.position.x, p.pose.position.y] for p in yellow_path.poses])
        if len(yellow_pts) == 0:
            return False, 0.0

        blue_right_count = 0
        for b_pose in blue_path.poses:
            b_x = b_pose.pose.position.x
            b_y = b_pose.pose.position.y

            distances = np.linalg.norm(yellow_pts - np.array([b_x, b_y]), axis=1)
            closest_idx = np.argmin(distances)
            y_y = yellow_pts[closest_idx][1]

            if b_y < y_y:
                blue_right_count += 1

        right_ratio = blue_right_count / len(blue_path.poses)
        return right_ratio > 0.5, right_ratio

    def compute_path_length(self, path_msg):
        if len(path_msg.poses) < 2:
            return 0.0

        total_length = 0.0
        for i in range(len(path_msg.poses) - 1):
            p0 = path_msg.poses[i].pose.position
            p1 = path_msg.poses[i + 1].pose.position
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            total_length += math.sqrt(dx*dx + dy*dy + dz*dz)

        return total_length

def main(args=None):
    rclpy.init(args=args)
    node = CenterlineGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()