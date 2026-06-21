import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class LanePathPublisher(Node):
    def __init__(self):
        super().__init__('lane_path_publisher')
        self.blue_points = []
        self.yellow_points = []
        self.max_point_step_m = 0.5
        self.positive_y_bias_m_per_m = 0.1
        self.positive_x_bias_m_per_m = 0.1

        self.sub_blue = self.create_subscription(
            PointCloud2, '/vision/line_points/blue', self.blue_callback, 10
        )
        # self.sub_yellow = self.create_subscription(
        #     PointCloud2, '/vision/line_points/yellow', self.yellow_callback, 10
        # )
        self.pub = self.create_publisher(Path, '/vision/path', 10)

    def blue_callback(self, msg: PointCloud2):
        self.blue_points = self.read_point_cloud(msg)
        self.publish_combined_path(msg.header)

    def yellow_callback(self, msg: PointCloud2):
        self.yellow_points = self.read_point_cloud(msg)
        self.publish_combined_path(msg.header)

    def publish_combined_path(self, header):
        all_points = self.blue_points + self.yellow_points
        self.pub.publish(self._to_path(all_points, header))

    def read_point_cloud(self, msg):
        points = []
        for point in point_cloud2.read_points(
            msg,
            field_names=('x', 'y'),
            skip_nans=True,
        ):
            # In recent ROS 2 versions, each point may be a structured scalar
            # (np.void), which supports named-field access but not slicing.
            try:
                x = float(point['x'])
                y = float(point['y'])
            except (TypeError, IndexError, KeyError):
                x = float(point[0])
                y = float(point[1])
            points.append((x, y))
        return self._sort_points_nearest_chain(points)

    def _sort_points_nearest_chain(self, points):
        if not points:
            return []

        remaining = list(points)

        # Start from the point nearest to the robot origin (0, 0).
        start_idx = min(
            range(len(remaining)),
            key=lambda i: remaining[i][0] * remaining[i][0] + remaining[i][1] * remaining[i][1],
        )
        ordered = [remaining.pop(start_idx)]

        # Greedily append the point nearest to the last selected point.
        while remaining:
            last_x, last_y = ordered[-1]
            next_idx = min(
                range(len(remaining)),
                key=lambda i: (
                    math.dist((last_x, last_y), remaining[i])
                    - self.positive_y_bias_m_per_m * remaining[i][1] # bias towards positive y
                    - self.positive_x_bias_m_per_m * remaining[i][0] # bias towards positive x
                ),
            )
            
            # if next point too far, stop
            next_point = remaining[next_idx]
            step_dist = math.dist((last_x, last_y), next_point)
            if step_dist > self.max_point_step_m:
                # self.get_logger().info(
                #     f'Stopping chain: next point distance {step_dist:.3f} m '
                #     f'> {self.max_point_step_m:.3f} m'
                # )
                break
            ordered.append(remaining.pop(next_idx))

        return ordered
    
    def _to_path(self, pts, header) -> Path:
        path = Path()
        path.header = header
        for x, y in pts:
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path
    
def main(args=None):
    rclpy.init(args=args)
    node = LanePathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()