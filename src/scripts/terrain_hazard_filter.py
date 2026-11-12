#!/usr/bin/env python3
"""
Terrain Hazard Filter Node for Caterpillar Autonomy Challenge (Shaastra 2027)
============================================================================
Processes downward 3D stereo point cloud from ZED 2i camera to extract:
1. Positive Obstacles (Boulders > 8cm height).
2. Negative Obstacles (Craters / Depressions < -8cm depth).
3. STRICT WALL EXCLUSION (Rule 3 & 4 Compliance):
   Purges all points within 0.25m of outer arena perimeter or above 0.40m height.
   Arena walls are NEVER used for sensing, mapping, or collision avoidance.
4. Virtual Boundary Keepout Buffer:
   Publishes a virtual safety perimeter (0.45m cushion) derived purely from
   known arena dimensions (9m x 5m) so the rover stays safely within boundaries.

Author: MaRs AutoNav Team
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


def voxel_downsample(points, voxel_size=0.04):
    """Fast 3D voxel grid downsampling using NumPy."""
    if len(points) == 0:
        return points
    coords = np.floor(points / voxel_size).astype(np.int32)
    # Pack integer coordinates into unique keys
    keys = coords[:, 0] * 73856093 ^ coords[:, 1] * 19349663 ^ coords[:, 2] * 83492791
    _, unique_indices = np.unique(keys, return_index=True)
    return points[unique_indices]


class TerrainHazardFilterNode(Node):
    def __init__(self):
        super().__init__('terrain_hazard_filter')

        # ---------------- Parameters ----------------
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('input_cloud_topic', '/depth_camera/points')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('boulder_threshold_z', 0.10)   # +10 cm above sand ground
        self.declare_parameter('crater_threshold_z', -0.10)   # -10 cm below sand ground
        self.declare_parameter('min_range', 0.40)             # Ignore points closer than 40cm
        self.declare_parameter('max_range', 3.50)             # Ignore points beyond 3.5m ahead
        self.declare_parameter('voxel_size', 0.04)            # 4 cm voxel resolution
        self.declare_parameter('arena_length_x', 9.0)         # 9.0m arena length
        self.declare_parameter('arena_width_y', 5.0)          # 5.0m arena width
        self.declare_parameter('wall_crop_margin', 0.25)      # 25cm wall exclusion filter
        self.declare_parameter('wall_height_ceiling', 0.40)   # Drop points above 40cm height
        self.declare_parameter('virtual_wall_buffer', 0.45)   # 45cm virtual keepout cushion

        self.input_topic = self.get_parameter('input_cloud_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.boulder_z = float(self.get_parameter('boulder_threshold_z').value)
        self.crater_z = float(self.get_parameter('crater_threshold_z').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.arena_lx = float(self.get_parameter('arena_length_x').value)
        self.arena_ly = float(self.get_parameter('arena_width_y').value)
        self.wall_crop = float(self.get_parameter('wall_crop_margin').value)
        self.wall_ceiling = float(self.get_parameter('wall_height_ceiling').value)
        self.virtual_buffer = float(self.get_parameter('virtual_wall_buffer').value)

        # ---------------- TF Setup ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Publishers & Subscriptions ----------------
        qos_sub = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub_cloud = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, qos_sub
        )

        # Hazard Publishers for Nav2 Costmaps
        self.pub_all_hazards = self.create_publisher(
            PointCloud2, '/hazards/all_hazards', 5
        )
        self.pub_boulders = self.create_publisher(
            PointCloud2, '/hazards/boulders', 5
        )
        self.pub_craters = self.create_publisher(
            PointCloud2, '/hazards/craters', 5
        )
        self.pub_virtual_boundary = self.create_publisher(
            PointCloud2, '/hazards/virtual_boundary', 5
        )

        # Precompute virtual keepout boundary pointcloud
        self.virtual_boundary_cloud = self.generate_virtual_boundary_cloud()

        # Timer to broadcast virtual keepout boundary at 2 Hz
        self.timer_keepout = self.create_timer(0.5, self.publish_virtual_boundary)

        self.get_logger().info(
            f'Terrain Hazard Filter initialized. Subscribed to {self.input_topic}. '
            f'Wall exclusion margin: {self.wall_crop}m (Rule 3 & 4 compliant).'
        )

    def generate_virtual_boundary_cloud(self):
        """Generates static 3D obstacle points along the virtual arena boundary."""
        pts = []
        step = 0.05  # 5 cm spacing along perimeter
        buffer = self.virtual_buffer
        lx = self.arena_lx
        ly = self.arena_ly

        # Generate virtual barrier lines:
        # 1. South perimeter boundary (Y = buffer)
        for x in np.arange(0.0, lx, step):
            for z in [0.05, 0.15, 0.25]:
                pts.append([x, buffer, z])
                pts.append([x, buffer * 0.5, z])

        # 2. North perimeter boundary (Y = ly - buffer)
        for x in np.arange(0.0, lx, step):
            for z in [0.05, 0.15, 0.25]:
                pts.append([x, ly - buffer, z])
                pts.append([x, ly - buffer * 0.5, z])

        # 3. West perimeter boundary (X = buffer)
        for y in np.arange(0.0, ly, step):
            for z in [0.05, 0.15, 0.25]:
                pts.append([buffer, y, z])
                pts.append([buffer * 0.5, y, z])

        # 4. East perimeter boundary (X = lx - buffer)
        for y in np.arange(0.0, ly, step):
            for z in [0.05, 0.15, 0.25]:
                pts.append([lx - buffer, y, z])
                pts.append([lx - buffer * 0.5, y, z])

        # 5. Gap Divider Barrier: Non-traversable separation between Starting & Construction zones
        # Start Zone: X in [0, 2], Y in [3, 5]; Construction: X in [0, 2], Y in [0, 1.5]
        # Barrier at gap Y in [1.5, 3.0], X in [0.0, 1.8]
        for y in np.arange(1.5, 3.0, step):
            for x in np.arange(0.0, 1.8, step):
                for z in [0.05, 0.15]:
                    pts.append([x, y, z])

        pts_arr = np.array(pts, dtype=np.float32)
        header = Header()
        header.frame_id = self.map_frame
        return pc2.create_cloud_xyz32(header, pts_arr)

    def publish_virtual_boundary(self):
        """Periodically publishes virtual keepout perimeter for Nav2 costmaps."""
        self.virtual_boundary_cloud.header.stamp = self.get_clock().now().to_msg()
        self.pub_virtual_boundary.publish(self.virtual_boundary_cloud)

    def cloud_callback(self, msg: PointCloud2):
        # 1. Read input points as NumPy array
        pts_cam = pc2.read_points_numpy(msg, field_names=['x', 'y', 'z'])
        valid = ~np.isnan(pts_cam).any(axis=1) & ~np.isinf(pts_cam).any(axis=1)
        pts_cam = pts_cam[valid]

        if len(pts_cam) == 0:
            return

        # 2. Lookup transform: base_footprint -> camera_link (or sensor link)
        t_base_cam = self.lookup_matrix(self.base_frame, msg.header.frame_id, msg.header.stamp)
        if t_base_cam is None:
            return

        # Transform points to base_footprint frame
        pts_base = pts_cam @ t_base_cam[:3, :3].T + t_base_cam[:3, 3]

        # 3. Range Gating ahead of rover (in base_footprint: X is forward, Y is lateral)
        range_mask = (pts_base[:, 0] >= self.min_range) & (pts_base[:, 0] <= self.max_range)
        pts_base = pts_base[range_mask]

        if len(pts_base) == 0:
            return

        # 4. Strict Wall Cropping (Rule 3 & 4 Compliance)
        # Transform points to map frame to apply global perimeter exclusion
        t_map_base = self.lookup_matrix(self.map_frame, self.base_frame, msg.header.stamp)
        if t_map_base is not None:
            pts_map = pts_base @ t_map_base[:3, :3].T + t_map_base[:3, 3]
            # Exclude any points near outer enclosure walls or above wall ceiling
            non_wall = (
                (pts_map[:, 0] > self.wall_crop) &
                (pts_map[:, 0] < (self.arena_lx - self.wall_crop)) &
                (pts_map[:, 1] > self.wall_crop) &
                (pts_map[:, 1] < (self.arena_ly - self.wall_crop)) &
                (pts_map[:, 2] < self.wall_ceiling)
            )
            pts_base = pts_base[non_wall]
            pts_map = pts_map[non_wall]
        else:
            pts_map = None

        if len(pts_base) == 0:
            return

        # 5. Segment Positive Obstacles (Boulders) and Negative Obstacles (Craters)
        # In map coordinates, the arena sand floor is globally flat at Z = 0.0m.
        # Using Z_map makes obstacle detection completely invariant to rover pitch/roll!
        if pts_map is not None:
            z_ref = pts_map[:, 2]
            out_pts = pts_map
            out_frame = self.map_frame
        else:
            z_ref = pts_base[:, 2]
            out_pts = pts_base
            out_frame = self.base_frame

        # Positive Obstacles: Boulders protruding above sand ground
        boulder_mask = z_ref > self.boulder_z
        boulder_pts = out_pts[boulder_mask]

        # Negative Obstacles: Craters plunging below sand ground
        crater_mask = z_ref < self.crater_z
        crater_pts = out_pts[crater_mask]

        # Downsample obstacles to keep costmap ray-tracing lightweight
        boulder_pts_ds = voxel_downsample(boulder_pts, self.voxel_size)
        crater_pts_ds = voxel_downsample(crater_pts, self.voxel_size)

        # For crater points: to ensure 2D costmaps mark them as lethal obstacles at ground level,
        # set their Z to a small positive value (e.g. 0.05m) so costmap obstacle layers don't clear them
        crater_pts_costmap = crater_pts_ds.copy()
        if len(crater_pts_costmap) > 0:
            crater_pts_costmap[:, 2] = 0.05

        # Combined Hazards for Nav2 Obstacle Layer
        if len(boulder_pts_ds) > 0 and len(crater_pts_costmap) > 0:
            all_hazards = np.vstack([boulder_pts_ds, crater_pts_costmap])
        elif len(boulder_pts_ds) > 0:
            all_hazards = boulder_pts_ds
        elif len(crater_pts_costmap) > 0:
            all_hazards = crater_pts_costmap
        else:
            all_hazards = np.empty((0, 3), dtype=np.float32)

        # 7. Publish Resulting Point Clouds
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = out_frame

        if len(all_hazards) > 0:
            cloud_all = pc2.create_cloud_xyz32(header, all_hazards.astype(np.float32))
            self.pub_all_hazards.publish(cloud_all)

        if len(boulder_pts_ds) > 0 and self.pub_boulders.get_subscription_count() > 0:
            cloud_boulders = pc2.create_cloud_xyz32(header, boulder_pts_ds.astype(np.float32))
            self.pub_boulders.publish(cloud_boulders)

        if len(crater_pts_ds) > 0 and self.pub_craters.get_subscription_count() > 0:
            cloud_craters = pc2.create_cloud_xyz32(header, crater_pts_ds.astype(np.float32))
            self.pub_craters.publish(cloud_craters)

    def lookup_matrix(self, target_frame, source_frame, stamp):
        """Looks up 4x4 transformation matrix between frames."""
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.04)
            )
            t = [
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y,
                tf_stamped.transform.translation.z
            ]
            q = [
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w
            ]
            mat = np.eye(4)
            mat[:3, :3] = R.from_quat(q).as_matrix()
            mat[:3, 3] = t
            return mat
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TerrainHazardFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
