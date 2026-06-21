import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import struct
from collections import defaultdict

# How many border points to place per edge
BORDER_DENSITY = 10  # points per edge
BIN_X = 0.2
BIN_Y = 0.3

def pack_rgb(r, g, b):
    """Pack r,g,b integers into a float32 the same way ROS PointCloud2 expects."""
    rgb_int = (r << 16) | (g << 8) | b
    return struct.unpack('f', struct.pack('I', rgb_int))[0]

RED_FLOAT = pack_rgb(255, 0, 0)

class BlueLineFilter(Node):
    def __init__(self):
        super().__init__('blue_line_filter')
        self.sub = self.create_subscription(
            PointCloud2, '/vision/line_points/blue', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/vision/line_points/blue_filtered', 10)

    def _bin_border_points(self, bx, by):
        """
        Generate points along the 4 edges of a bin cell in world coordinates.
        Each bin spans [bx*BIN_X, (bx+1)*BIN_X] x [by*BIN_Y, (by+1)*BIN_Y].
        Returns a list of (x, y, z, rgb) tuples with z=0 and rgb=red.
        """
        x_min = bx * BIN_X
        x_max = (bx + 1) * BIN_X
        y_min = by * BIN_Y
        y_max = (by + 1) * BIN_Y

        border = []

        # Bottom and top edges (vary x, fix y)
        for t in np.linspace(x_min, x_max, BORDER_DENSITY):
            border.append((t, y_min, 0.0, RED_FLOAT))
            border.append((t, y_max, 0.0, RED_FLOAT))

        # Left and right edges (vary y, fix x)
        for t in np.linspace(y_min, y_max, BORDER_DENSITY):
            border.append((x_min, t, 0.0, RED_FLOAT))
            border.append((x_max, t, 0.0, RED_FLOAT))

        return border

    def callback(self, msg: PointCloud2):
        # Read ALL point data and store as tuples
        full_points = []
        for p in point_cloud2.read_points(msg, skip_nans=True):
            full_points.append(tuple(p))

        if not full_points:
            return

        # Extract x,y for spatial processing
        pts = np.array([(p[0], p[1]) for p in full_points], dtype=np.float64)

        # Step 1: Bin by grid
        bins = defaultdict(list)
        for i in range(pts.shape[0]):
            x, y = pts[i]
            bx = int(np.floor(x / BIN_X))
            by = int(np.floor(y / BIN_Y))
            bins[(bx, by)].append(i)

        # Step 2: For each bin, pick point with maximum y
        candidate_indices = []
        candidate_bin_keys = []  # track which bin each candidate came from
        for (bx, by), indices in bins.items():
            bin_pts = pts[indices]
            max_y_idx = indices[np.argmax(bin_pts[:, 1])]
            candidate_indices.append(max_y_idx)
            candidate_bin_keys.append((bx, by))

        # Step 3: Remove isolated points
        candidates = pts[candidate_indices]
        keep_mask = np.zeros(len(candidate_indices), dtype=bool)

        for i in range(len(candidate_indices)):
            diffs = candidates - candidates[i]
            dists = np.sqrt(np.sum(diffs**2, axis=1))
            neighbor_count = np.sum((dists > 0) & (dists <= 1.5))
            if neighbor_count >= 2:
                keep_mask[i] = True

        final_indices = [candidate_indices[i] for i in np.where(keep_mask)[0]]
        # Only draw borders for bins whose candidate survived the isolation filter
        surviving_bins = [candidate_bin_keys[i] for i in np.where(keep_mask)[0]]

        if not final_indices:
            return

        # Step 4: Build filtered point list + red border points
        filtered_points = [full_points[i] for i in final_indices]

        # Append red border points for each surviving bin
        for (bx, by) in surviving_bins:
            filtered_points.extend(self._bin_border_points(bx, by))

        new_msg = point_cloud2.create_cloud(msg.header, msg.fields, filtered_points)
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BlueLineFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()