import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from rclpy.executors import MultiThreadedExecutor


class PointCloudBinner(Node):
    def __init__(self, colour):
        super().__init__('pointcloud_binner')

        # self.declare_parameter('colour', 'yellow')
        self.declare_parameter('chunk_size_x_m', 0.10)
        self.declare_parameter('chunk_size_y_m', 0.10)

        self.colour = colour # str(self.get_parameter('colour').value)
        self.chunk_size_x_m = float(self.get_parameter('chunk_size_x_m').value)
        self.chunk_size_y_m = float(self.get_parameter('chunk_size_y_m').value)

        self.input_topic = f'/ipm_cloud/{self.colour}'
        self.output_topic = f'/vision/point_cloud_binned/{self.colour}'

        self.sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.callback,
            10,
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f'PointCloud binner started for colour={self.colour}. '
            f'input={self.input_topic}, output={self.output_topic}, '
            f'chunk={self.chunk_size_x_m:.3f}x{self.chunk_size_y_m:.3f} m'
        )

    def callback(self, msg: PointCloud2):
        field_names = [f.name for f in msg.fields]
        required_fields = {'x', 'y', 'z'}
        if not required_fields.issubset(set(field_names)):
            self.get_logger().warn(
                f'Skipping cloud: expected fields {sorted(required_fields)}, got {field_names}'
            )
            return

        points_raw = list(
            point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
        )
        if len(points_raw) == 0:
            return

        points_struct = np.array(points_raw)
        xyz = np.stack([
            points_struct['x'].astype(np.float32),
            points_struct['y'].astype(np.float32),
            points_struct['z'].astype(np.float32),
        ], axis=1)

        binned_xyz = self._chunk_points_xy_mean(xyz)
        if binned_xyz.shape[0] == 0:
            return

        out_msg = point_cloud2.create_cloud_xyz32(
            header=msg.header,
            points=binned_xyz,
        )
        self.pub.publish(out_msg)

    def _chunk_points_xy_mean(self, xyz: np.ndarray):
        """
        Aggregate points into XY grid cells and return one mean point per cell.
        Cell size is configurable via self.chunk_size_x_m and self.chunk_size_y_m.
        """
        if xyz.shape[0] == 0:
            return xyz

        if self.chunk_size_x_m <= 0.0 or self.chunk_size_y_m <= 0.0:
            return xyz

        bin_x = np.floor(xyz[:, 0] / self.chunk_size_x_m).astype(np.int64)
        bin_y = np.floor(xyz[:, 1] / self.chunk_size_y_m).astype(np.int64)
        bin_ids = np.stack([bin_x, bin_y], axis=1)

        _, inverse = np.unique(bin_ids, axis=0, return_inverse=True)
        n_bins = int(np.max(inverse)) + 1

        counts = np.bincount(inverse, minlength=n_bins).astype(np.float32)

        sum_x = np.bincount(inverse, weights=xyz[:, 0], minlength=n_bins)
        sum_y = np.bincount(inverse, weights=xyz[:, 1], minlength=n_bins)
        sum_z = np.bincount(inverse, weights=xyz[:, 2], minlength=n_bins)
        mean_xyz = np.stack([sum_x / counts, sum_y / counts, sum_z / counts], axis=1).astype(np.float32)

        return mean_xyz


def main(args=None):
    rclpy.init(args=args)
    node_yellow = PointCloudBinner('yellow')
    node_blue = PointCloudBinner('blue')
    executor = MultiThreadedExecutor()
    executor.add_node(node_yellow)
    executor.add_node(node_blue)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node_yellow.destroy_node()
        node_blue.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
