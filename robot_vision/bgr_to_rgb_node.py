import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np

class BGRtoRGBConverter(Node):
    def __init__(self):
        super().__init__('bgr_to_rgb_converter')
        self.chunk_size_x_m = 0.10
        self.chunk_size_y_m = 0.10
        
        self.declare_parameter('colour', 'yellow')
        self.colour = self.get_parameter('colour').value

        self.sub = self.create_subscription(
            PointCloud2, f'/ipm_cloud/{self.colour}', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, f'/vision/point_cloud_rgb/{self.colour}', 10)

    def callback(self, msg):
        points_raw = list(
            point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'rgb'),
                skip_nans=True
            )
        )
        if len(points_raw) == 0:
            return

        # Build dense numeric arrays from the incoming cloud.
        xyz = np.array([(p[0], p[1], p[2]) for p in points_raw], dtype=np.float32)
        rgb_in = np.array([p[3] for p in points_raw])

        # Handle rgb provided either as uint32 or as float32 bit-pattern.
        if np.issubdtype(rgb_in.dtype, np.floating):
            rgb_bgr = rgb_in.astype(np.float32).view(np.uint32)
        else:
            rgb_bgr = rgb_in.astype(np.uint32)

        # Convert packed BGR (0x00BBGGRR) -> RGB (0x00RRGGBB).
        b = (rgb_bgr >> 16) & 0xFF
        g = (rgb_bgr >> 8) & 0xFF
        r = rgb_bgr & 0xFF
        rgb_rgb = ((r << 16) | (g << 8) | b).astype(np.uint32)

        chunked_xyz, chunked_rgb = self._chunk_points_xy_mean(xyz, rgb_rgb)
        if chunked_xyz.shape[0] == 0:
            return

        cloud_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32),
        ])

        out_struct = np.zeros(chunked_xyz.shape[0], dtype=cloud_dtype)
        out_struct['x'] = chunked_xyz[:, 0]
        out_struct['y'] = chunked_xyz[:, 1]
        out_struct['z'] = chunked_xyz[:, 2]
        out_struct['rgb'] = chunked_rgb

        out_msg = point_cloud2.create_cloud(
            header=msg.header,
            fields=msg.fields,
            points=out_struct
        )
        self.pub.publish(out_msg)

    def _chunk_points_xy_mean(self, xyz: np.ndarray, rgb_packed: np.ndarray):
        """
        Aggregate points into XY grid cells and return one mean point per cell.
        Cell size is configurable via self.chunk_size_x_m and self.chunk_size_y_m.
        """
        if xyz.shape[0] == 0:
            return xyz, rgb_packed

        if self.chunk_size_x_m <= 0.0 or self.chunk_size_y_m <= 0.0:
            return xyz, rgb_packed

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

        r = ((rgb_packed >> 16) & 0xFF).astype(np.float32)
        g = ((rgb_packed >> 8) & 0xFF).astype(np.float32)
        b = (rgb_packed & 0xFF).astype(np.float32)

        mean_r = np.clip(np.rint(np.bincount(inverse, weights=r, minlength=n_bins) / counts), 0, 255).astype(np.uint32)
        mean_g = np.clip(np.rint(np.bincount(inverse, weights=g, minlength=n_bins) / counts), 0, 255).astype(np.uint32)
        mean_b = np.clip(np.rint(np.bincount(inverse, weights=b, minlength=n_bins) / counts), 0, 255).astype(np.uint32)
        mean_rgb = ((mean_r << 16) | (mean_g << 8) | mean_b).astype(np.uint32)

        return mean_xyz, mean_rgb

def main(args=None):
    rclpy.init(args=args)
    node = BGRtoRGBConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()