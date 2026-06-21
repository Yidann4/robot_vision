"""
line_filter_node.py — Colour-Based Line Filter
===============================================
Subscribes to:
  /projected_point_cloud     (sensor_msgs/PointCloud2)  xyzrgb

Publishes:
    /line_points               (sensor_msgs/PointCloud2)  xyzrgb
    → Only points whose colour falls within the configured HSV range.
      All positions are already in the base_footprint frame (global
      coordinates) — no conversion needed.

How colour filtering works:
  Each point in the cloud carries a packed RGB value. We unpack it,
  convert to HSV (Hue-Saturation-Value), and keep only points whose
  hue falls within the target range. HSV is used instead of RGB because
  it is far more robust to lighting changes — the hue of "yellow" stays
  roughly the same whether the light is bright or dim, while the RGB
  values shift dramatically.

TUNING — the only thing you need to adjust:
  The HSV_RANGES dict below defines which colours to keep.
  Each entry is (h_min, h_max, s_min, s_max, v_min, v_max).
  OpenCV HSV ranges: H=0-179, S=0-255, V=0-255.

  To find the right values for your specific lines:
    1. Run this node with the defaults
    2. View /line_points in RViz
    3. If too many non-line points are included → tighten the ranges
    4. If line points are being dropped → widen the ranges
  Or use the HSV tuner script described at the bottom of this file.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import numpy as np
import struct


# ---------------------------------------------------------------------------
# COLOUR CONFIGURATION
# ---------------------------------------------------------------------------
# Add or remove colours to match your floor lines.
# Format per entry:  (h_min, h_max, s_min, s_max, v_min, v_max)
# OpenCV HSV:  H 0-179,  S 0-255,  V 0-255
#
# Common starting points:
#   Yellow:  H 20-35,   S 100-255, V 100-255
#   Red:     H 0-10  +  H 160-179 (red wraps around 0 in HSV)
#   Blue:    H 100-130, S 100-255, V 50-255
#   Green:   H 40-80,   S 100-255, V 50-255
#   White:   H 0-179,   S 0-40,    V 200-255  (low saturation, high value)
# ---------------------------------------------------------------------------

DEBUG = 1

HSV_RANGES = {
    'yellow': (8, 70, 0, 255, 0, 255),
    'blue': (87, 179, 0, 255, 0, 255),
}

# Minimum number of points a colour cluster must have to be published.
# Filters out single stray pixels caused by noise.
MIN_CLUSTER_POINTS = 0


class LineFilterNode(Node):

    def __init__(self):
        super().__init__('line_filter_node')
        self.get_logger().info('Line filter node starting...')
        self.chunk_size_x_m = 0.10
        self.chunk_size_y_m = 0.10

        # Subscribe to the full projected point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            '/vision/projected_point_cloud_rgb',
            self.callback,
            10
        )

        # Publish combined filtered line points
        self.pub = self.create_publisher(PointCloud2, '/vision/line_points', 10)

        self.get_logger().info(
            f'Subscribing to /vision/projected_point_cloud_rgb\n'
            f'Publishing filtered lines to /vision/line_points\n'
            f'Active colour filters: {list(HSV_RANGES.keys())}'
        )

    def callback(self, msg: PointCloud2):
        """
        Called on every incoming point cloud frame.
        Extracts xyzrgb points, filters by colour, republishes matches.
        """

        # ---- Step 1: unpack the point cloud into numpy arrays ----
        # read_points gives us an iterator of named tuples (x, y, z, rgb)
        # We convert to a structured numpy array for fast vectorised operations.
        points_raw = list(
            point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z', 'rgb'),
                skip_nans=True
            )
        )
        if DEBUG:
            self.get_logger().debug(f'Processing cloud with {len(points_raw)} points')

        if len(points_raw) == 0:
            return

        # read_points returns a structured array with mixed dtypes —
        # (x: float32, y: float32, z: float32, rgb: uint32).
        # We cannot cast the whole thing to float32, so we extract each
        # field by name into its own array instead.
        points_struct = np.array(points_raw)   # structured array, shape (N,)

        xyz = np.stack([
            points_struct['x'].astype(np.float32),
            points_struct['y'].astype(np.float32),
            points_struct['z'].astype(np.float32),
        ], axis=1)                              # shape (N, 3)

        # rgb is stored as uint32 packed as 0x00RRGGBB
        rgb_packed = points_struct['rgb'].astype(np.uint32)  # shape (N,)

        # ---- Step 2: unpack RGB bytes from the packed uint32 ----
        r = ((rgb_packed >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_packed >>  8) & 0xFF).astype(np.uint8)
        b = ((rgb_packed      ) & 0xFF).astype(np.uint8)

        # Stack into (N, 1, 3) — the shape cv2.cvtColor expects
        # We do this without importing cv2 by doing the RGB→HSV conversion
        # ourselves using numpy, keeping the node lightweight.
        hsv = self._rgb_to_hsv_numpy(r, g, b)   # shape (N, 3)  H S V

        # ---- Step 3: build one mask per configured colour ----
        colour_masks = {}
        for name, (h_min, h_max, s_min, s_max, v_min, v_max) in HSV_RANGES.items():
            mask = (
                (hsv[:, 0] >= h_min) & (hsv[:, 0] <= h_max) &
                (hsv[:, 1] >= s_min) & (hsv[:, 1] <= s_max) &
                (hsv[:, 2] >= v_min) & (hsv[:, 2] <= v_max)
            )
            colour_masks[name] = mask
            n_matched = int(np.sum(mask))
            if n_matched > 0:
                self.get_logger().debug(
                    f'Colour [{name}]: {n_matched} points matched'
                )

        keep_blue = colour_masks.get('blue', np.zeros(len(points_raw), dtype=bool))
        keep_yellow = colour_masks.get('yellow', np.zeros(len(points_raw), dtype=bool))
        keep_combined = keep_blue | keep_yellow

        # ---- Step 4: publish one combined stream ----
        self._publish_filtered_cloud(
            keep_combined,
            xyz,
            rgb_packed,
            msg,
            self.pub,
            'combined'
        )

    def _publish_filtered_cloud(self, keep, xyz, rgb_packed, msg, publisher, colour_name):
        total_kept = int(np.sum(keep))
        if total_kept < MIN_CLUSTER_POINTS:
            self.get_logger().debug(
                f'Only {total_kept} {colour_name} points matched — below minimum, skipping'
            )
            return

        if total_kept == 0:
            return

        if DEBUG:
            self.get_logger().debug(f'Publishing {total_kept} {colour_name} line points')

        filtered_xyz = xyz[keep]                                    # shape (M, 3) float32
        filtered_rgb = rgb_packed[keep]

        filtered_xyz, filtered_rgb = self._chunk_points_xy_mean(filtered_xyz, filtered_rgb)
        total_out = int(filtered_xyz.shape[0])
        if total_out == 0:
            return

        if DEBUG:
            self.get_logger().debug(
                f'Chunked {colour_name}: {total_kept} raw points -> {total_out} mean points '
                f'using {self.chunk_size_x_m:.3f}x{self.chunk_size_y_m:.3f} m cells'
            )

        # Create a structured numpy array that perfectly matches the cloud layout.
        # This completely avoids Python loop overhead and NaN casting errors.
        cloud_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)  # Keep it as uint32 here matching the raw byte layout
        ])

        filtered_struct = np.zeros(total_out, dtype=cloud_dtype)
        filtered_struct['x'] = filtered_xyz[:, 0]
        filtered_struct['y'] = filtered_xyz[:, 1]
        filtered_struct['z'] = filtered_xyz[:, 2]
        filtered_struct['rgb'] = filtered_rgb

        # Use the same fields definition as the input cloud
        out_msg = point_cloud2.create_cloud(
            header=msg.header,   # preserves frame_id and timestamp
            fields=msg.fields,
            points=filtered_struct
        )

        publisher.publish(out_msg)

    def _chunk_points_xy_mean(self, xyz: np.ndarray, rgb_packed: np.ndarray):
        """
        Aggregate points into XY grid cells and return one mean point per cell.
        Cell size is configurable via self.chunk_size_x_m and self.chunk_size_y_m.
        """
        return xyz, rgb_packed  # disable chunking for now
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

    @staticmethod
    def _rgb_to_hsv_numpy(r: np.ndarray, g: np.ndarray, b: np.ndarray) -> np.ndarray:
        """
        Convert R, G, B uint8 arrays to HSV without requiring OpenCV.
        Returns array of shape (N, 3) with H in [0,179], S and V in [0,255]
        to match OpenCV's HSV convention (used in HSV_RANGES above).

        This avoids a per-frame Python loop and runs entirely in numpy,
        making it fast enough for real-time point cloud processing.
        """
        # Normalise to [0, 1]
        rf = r.astype(np.float32) / 255.0
        gf = g.astype(np.float32) / 255.0
        bf = b.astype(np.float32) / 255.0

        cmax = np.maximum(np.maximum(rf, gf), bf)   # V channel
        cmin = np.minimum(np.minimum(rf, gf), bf)
        diff = cmax - cmin                           # chroma

        # --- Hue ---
        h = np.zeros_like(rf)

        # Avoid division by zero — only compute hue where chroma > 0
        mask_r = (cmax == rf) & (diff > 0)
        mask_g = (cmax == gf) & (diff > 0)
        mask_b = (cmax == bf) & (diff > 0)

        h[mask_r] = (60.0 * ((gf[mask_r] - bf[mask_r]) / diff[mask_r]) % 360)
        h[mask_g] = (60.0 * ((bf[mask_g] - rf[mask_g]) / diff[mask_g]) + 120)
        h[mask_b] = (60.0 * ((rf[mask_b] - gf[mask_b]) / diff[mask_b]) + 240)

        # Bring into [0, 360) then scale to OpenCV's [0, 179]
        h = h % 360.0
        h_cv = (h / 2.0).astype(np.uint8)

        # --- Saturation ---
        s = np.where(cmax > 0, diff / cmax, 0.0)
        s_cv = (s * 255.0).astype(np.uint8)

        # --- Value ---
        v_cv = (cmax * 255.0).astype(np.uint8)

        return np.stack([h_cv, s_cv, v_cv], axis=1)   # shape (N, 3)


def main(args=None):
    rclpy.init(args=args)
    node = LineFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ---------------------------------------------------------------------------
# HOW TO TUNE YOUR HSV RANGES
# ---------------------------------------------------------------------------
# If the defaults don't work well for your lighting conditions, the fastest
# way to find the right values is to sample directly from your point cloud:
#
#   ros2 topic echo /projected_point_cloud --once > cloud.txt
#
# Or run this one-liner to print the HSV of points in a specific area:
#
#   ros2 run robot_vision line_filter_node
#   ros2 topic echo /line_points --once | head -20
#
# Then adjust HSV_RANGES until /line_points in RViz shows only your lines.
#
# Rule of thumb for tightening:
#   - Too much noise included?  → raise s_min (require more saturated colour)
#   - Lines dropping out?       → lower s_min or widen h range by ±5
#   - Shadows causing problems? → lower v_min
# ---------------------------------------------------------------------------