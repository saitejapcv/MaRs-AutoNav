#!/usr/bin/env python3
"""
ArUco Visual Localizer for Caterpillar Autonomy Challenge (Shaastra 2027)
========================================================================
Implements fiducial state estimation strictly complying with competition rules:
- Passive ArUco visual tags mounted ONLY on starting zone perimeter walls.
- Zero dependency on GPS, compass/magnetometer, or arena wall scanning.
- Performs:
    1. Initial Pose Lock in the Starting Zone -> Publishes /initialpose.
    2. Continuous drift-free map -> odom coordinate frame transform.
    3. Return-trip drift reset: corrects odometry when tags re-enter FOV
       before crossing the autonomous boundary into the Construction Zone.

Author: MaRs AutoNav Team
"""

import math
import numpy as np
import cv2
import cv2.aruco as aruco

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R


class ArucoLocalizerNode(Node):
    def __init__(self):
        super().__init__('aruco_localizer')

        # ---------------- Parameters ----------------
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_optical_link')
        self.declare_parameter('marker_size', 0.192)  # 19.2 cm black marker (within 24cm plane with white border)
        self.declare_parameter('aruco_dict_id', 0)   # DICT_4X4_50
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('tf_broadcast_rate', 30.0)
        self.declare_parameter('ema_alpha', 0.20)    # Smoothing filter factor
        self.declare_parameter('lock_threshold_count', 3)  # Consecutive frames to lock start pose
        self.declare_parameter('default_start_x', 1.0)     # Starting Zone default X
        self.declare_parameter('default_start_y', 4.0)     # Starting Zone default Y
        self.declare_parameter('default_start_yaw', 0.0)   # Starting Zone default Yaw

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_size = float(self.get_parameter('marker_size').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.ema_alpha = float(self.get_parameter('ema_alpha').value)
        self.lock_threshold = int(self.get_parameter('lock_threshold_count').value)

        # ---------------- OpenCV ArUco Setup ----------------
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 3D Corner coordinates of planar marker in marker local frame:
        # local X: Right (+), local Y: Up (+), local Z: Normal out towards camera (+)
        half_s = self.marker_size / 2.0
        self.obj_pts = np.array([
            [-half_s,  half_s, 0.0],  # 0: Top-Left
            [ half_s,  half_s, 0.0],  # 1: Top-Right
            [ half_s, -half_s, 0.0],  # 2: Bottom-Right
            [-half_s, -half_s, 0.0]   # 3: Bottom-Left
        ], dtype=np.float32)

        # ---------------- Known Marker Poses in Map Frame ----------------
        # Derived from cat_arena.sdf geometry (Starting Zone perimeter walls)
        # West Wall tags (Normal = +X, Up = +Z, Right = +Y)
        # North Wall tags (Normal = -Y, Up = +Z, Right = +X)
        self.TAG_POSES = {
            0: {
                'pos': np.array([0.005, 4.000, 0.350]),
                'rot': np.array([[0.0, 0.0, 1.0],
                                 [1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0]], dtype=float),
                'desc': 'West Wall Starting Zone Center'
            },
            1: {
                'pos': np.array([1.000, 4.995, 0.350]),
                'rot': np.array([[1.0,  0.0, 0.0],
                                 [0.0,  0.0, -1.0],
                                 [0.0,  1.0, 0.0]], dtype=float),
                'desc': 'North Wall Starting Zone Center'
            },
            2: {
                'pos': np.array([1.800, 4.995, 0.350]),
                'rot': np.array([[1.0,  0.0, 0.0],
                                 [0.0,  0.0, -1.0],
                                 [0.0,  1.0, 0.0]], dtype=float),
                'desc': 'North Wall Obstacle Zone Boundary'
            }
        }

        # ---------------- Camera Intrinsics ----------------
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_camera_info = False

        # Fallback Intrinsics matching cat_rover URDF (960x540, HFov 1.92 rad ~110 deg)
        fov_h = 1.92
        width = 960.0
        height = 540.0
        fx = (width / 2.0) / math.tan(fov_h / 2.0)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        self.fallback_camera_matrix = np.array([
            [fx,  0.0, cx],
            [0.0, fy,  cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.fallback_dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # ---------------- TF & State Estimation ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.has_initial_lock = False
        self.consecutive_locks = 0
        self.candidate_initial_poses = []

        # Current estimated map -> odom transform: [x, y, z, qx, qy, qz, qw]
        # Initialized with Starting Zone nominal pose (1.0, 4.0) so map -> odom is valid at startup
        def_x = float(self.get_parameter('default_start_x').value)
        def_y = float(self.get_parameter('default_start_y').value)
        def_yaw = float(self.get_parameter('default_start_yaw').value)
        self.map_to_odom_trans = np.array([def_x, def_y, 0.0])
        self.map_to_odom_quat = R.from_euler('xyz', [0.0, 0.0, def_yaw]).as_quat()
        self.tf_initialized = True

        # ---------------- Subscriptions & Publishers ----------------
        img_topic = self.get_parameter('camera_image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        self.sub_info = self.create_subscription(
            CameraInfo, info_topic, self.camera_info_callback, qos_profile_sensor_data)
        self.sub_img = self.create_subscription(
            Image, img_topic, self.image_callback, qos_profile_sensor_data)

        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.pub_rover_pose = self.create_publisher(
            PoseStamped, '/aruco/rover_pose', 10)
        self.pub_debug_img = self.create_publisher(
            Image, '/aruco/image_debug', 2)
        self.pub_markers = self.create_publisher(
            MarkerArray, '/aruco/detected_markers', 10)

        # Continuous broadcast timer for map -> odom
        rate = self.get_parameter('tf_broadcast_rate').value
        self.timer = self.create_timer(1.0 / rate, self.broadcast_tf_callback)

        self.get_logger().info('ArUco Localizer Node initialized. Waiting for camera stream...')

    def camera_info_callback(self, msg: CameraInfo):
        if not self.has_camera_info:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            if self.dist_coeffs.size == 0:
                self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
            self.has_camera_info = True
            self.get_logger().info(f'CameraInfo received: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}, cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}')

    def get_camera_intrinsics(self):
        if self.has_camera_info and self.camera_matrix is not None:
            return self.camera_matrix, self.dist_coeffs
        return self.fallback_camera_matrix, self.fallback_dist_coeffs

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            if self.publish_debug and self.pub_debug_img.get_subscription_count() > 0:
                self.pub_debug_img.publish(msg)
            return

        cam_mat, dist_coeff = self.get_camera_intrinsics()
        debug_img = cv_img.copy() if self.publish_debug else None

        # Look up transform from base_footprint to camera_optical_link
        t_base_cam = self.lookup_transform(self.base_frame, self.camera_frame, msg.header.stamp)
        if t_base_cam is None:
            # Fallback to static values if TF buffer is not ready yet
            t_base_cam = np.eye(4)
            # Mast (0.21, 0, 0.64) + pitch 20 deg + optical frame rotation
            r_pitch = R.from_euler('y', 0.35).as_matrix()
            r_opt = R.from_euler('xyz', [-math.pi/2, 0, -math.pi/2]).as_matrix()
            t_base_cam[:3, :3] = r_pitch @ r_opt
            t_base_cam[:3, 3] = [0.21, 0.0, 0.64]

        # Process each detected tag
        candidate_base_poses = []
        weights = []

        for i, tag_id in enumerate(ids.flatten()):
            if tag_id not in self.TAG_POSES:
                continue

            # Solve PnP for marker pose relative to camera
            pts_2d = corners[i].reshape((4, 2)).astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(
                self.obj_pts, pts_2d, cam_mat, dist_coeff, flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if not success:
                continue

            # Draw visual debug axes on image
            if debug_img is not None:
                cv2.drawFrameAxes(debug_img, cam_mat, dist_coeff, rvec, tvec, self.marker_size * 0.75)
                aruco.drawDetectedMarkers(debug_img, corners, ids)

            # Homogeneous transform T_camera_tag
            r_mat, _ = cv2.Rodrigues(rvec)
            t_cam_tag = np.eye(4)
            t_cam_tag[:3, :3] = r_mat
            t_cam_tag[:3, 3] = tvec.ravel()

            # Known tag pose in map
            tag_def = self.TAG_POSES[tag_id]
            t_map_tag = np.eye(4)
            t_map_tag[:3, :3] = tag_def['rot']
            t_map_tag[:3, 3] = tag_def['pos']

            # T_map_cam = T_map_tag * inv(T_cam_tag)
            t_map_cam = t_map_tag @ np.linalg.inv(t_cam_tag)

            # T_map_base = T_map_cam * inv(T_base_cam)
            t_map_base = t_map_cam @ np.linalg.inv(t_base_cam)

            # Distance weight: closer markers have higher accuracy
            dist = np.linalg.norm(tvec)
            if dist < 0.1:
                continue
            weight = 1.0 / (dist * dist)

            candidate_base_poses.append(t_map_base)
            weights.append(weight)

        if not candidate_base_poses:
            return

        # Weighted fusion of candidate poses
        weights = np.array(weights)
        weights /= np.sum(weights)

        fused_pos = np.zeros(3)
        quats = []
        for pose, w in zip(candidate_base_poses, weights):
            fused_pos += w * pose[:3, 3]
            r = R.from_matrix(pose[:3, :3])
            quats.append(r.as_quat())

        # Constrain to 2D Ground Plane (Z=0, Roll=0, Pitch=0)
        # Average yaw angle across detections
        yaws = [R.from_quat(q).as_euler('xyz')[2] for q in quats]
        mean_sin = np.sum([w * math.sin(y) for w, y in zip(weights, yaws)])
        mean_cos = np.sum([w * math.cos(y) for w, y in zip(weights, yaws)])
        fused_yaw = math.atan2(mean_sin, mean_cos)

        fused_rot = R.from_euler('xyz', [0.0, 0.0, fused_yaw])
        t_map_base_fused = np.eye(4)
        t_map_base_fused[:3, :3] = fused_rot.as_matrix()
        t_map_base_fused[:3, 3] = [fused_pos[0], fused_pos[1], 0.0]

        # Publish estimated rover pose topic
        self.publish_rover_pose(t_map_base_fused, msg.header.stamp)

        # Publish debug visualization image
        if debug_img is not None and self.pub_debug_img.get_subscription_count() > 0:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            dbg_msg.header = msg.header
            self.pub_debug_img.publish(dbg_msg)

        # ---------------- START ZONE INITIAL POSE LOCK ----------------
        if not self.has_initial_lock:
            self.candidate_initial_poses.append(t_map_base_fused)
            self.consecutive_locks += 1
            self.get_logger().info(
                f'[Pose Lock Progress {self.consecutive_locks}/{self.lock_threshold}] '
                f'Candidate: X={fused_pos[0]:.3f}m, Y={fused_pos[1]:.3f}m, Yaw={math.degrees(fused_yaw):.1f}°'
            )

            if self.consecutive_locks >= self.lock_threshold:
                # Average initial candidates
                init_x = np.mean([p[0, 3] for p in self.candidate_initial_poses])
                init_y = np.mean([p[1, 3] for p in self.candidate_initial_poses])
                yaws_init = [R.from_matrix(p[:3, :3]).as_euler('xyz')[2] for p in self.candidate_initial_poses]
                init_yaw = math.atan2(np.mean(np.sin(yaws_init)), np.mean(np.cos(yaws_init)))

                t_init = np.eye(4)
                t_init[:3, :3] = R.from_euler('xyz', [0.0, 0.0, init_yaw]).as_matrix()
                t_init[:3, 3] = [init_x, init_y, 0.0]

                self.publish_initial_pose(t_init, msg.header.stamp)
                self.update_map_to_odom(t_init, msg.header.stamp, is_initial=True)
                self.has_initial_lock = True
                self.get_logger().info(
                    f'>>> [SUCCESS] ARUCO POSE LOCKED IN STARTING ZONE: '
                    f'X={init_x:.3f}m, Y={init_y:.3f}m, Yaw={math.degrees(init_yaw):.1f}° <<<'
                )
        else:
            # ---------------- RETURN TRIP DRIFT CORRECTION ----------------
            # Rover re-observed markers on return from excavation zone
            # Smoothly update map -> odom using Exponential Moving Average
            self.update_map_to_odom(t_map_base_fused, msg.header.stamp, is_initial=False)

    def update_map_to_odom(self, t_map_base, stamp, is_initial=False):
        """
        Computes T_map_odom = T_map_base * inv(T_odom_base)
        and updates the broadcasted transform smoothly.
        """
        t_odom_base = self.lookup_transform(self.odom_frame, self.base_frame, stamp)
        if t_odom_base is None:
            # If odom frame is not yet publishing, assume identity
            t_odom_base = np.eye(4)

        t_map_odom = t_map_base @ np.linalg.inv(t_odom_base)

        target_trans = t_map_odom[:3, 3]
        target_rot = R.from_matrix(t_map_odom[:3, :3])
        target_quat = target_rot.as_quat()

        if is_initial or not self.tf_initialized:
            self.map_to_odom_trans = target_trans
            self.map_to_odom_quat = target_quat
            self.tf_initialized = True
        else:
            # Exponential Moving Average for position
            alpha = self.ema_alpha
            self.map_to_odom_trans = (1.0 - alpha) * self.map_to_odom_trans + alpha * target_trans

            # Spherical Linear Interpolation (SLERP) for orientation
            r_cur = R.from_quat(self.map_to_odom_quat)
            r_tgt = R.from_quat(target_quat)
            # Rotation difference
            r_diff = r_cur.inv() * r_tgt
            rotvec = r_diff.as_rotvec()
            # Step by alpha along rotation vector
            r_step = R.from_rotvec(alpha * rotvec)
            r_new = r_cur * r_step
            self.map_to_odom_quat = r_new.as_quat()

    def broadcast_tf_callback(self):
        """Continuously broadcasts map -> odom transform at fixed rate."""
        if not self.tf_initialized:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = float(self.map_to_odom_trans[0])
        t.transform.translation.y = float(self.map_to_odom_trans[1])
        t.transform.translation.z = 0.0

        t.transform.rotation.x = float(self.map_to_odom_quat[0])
        t.transform.rotation.y = float(self.map_to_odom_quat[1])
        t.transform.rotation.z = float(self.map_to_odom_quat[2])
        t.transform.rotation.w = float(self.map_to_odom_quat[3])

        self.tf_broadcaster.sendTransform(t)

    def publish_initial_pose(self, t_map_base, stamp):
        """Publishes /initialpose for Nav2 and AMCL compatibility."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = float(t_map_base[0, 3])
        msg.pose.pose.position.y = float(t_map_base[1, 3])
        msg.pose.pose.position.z = 0.0

        r = R.from_matrix(t_map_base[:3, :3])
        q = r.as_quat()
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        # High confidence initial covariance: 1cm position, 0.01 rad yaw
        cov = [0.0] * 36
        cov[0] = 0.01   # X
        cov[7] = 0.01   # Y
        cov[14] = 0.001 # Z
        cov[21] = 0.001 # roll
        cov[28] = 0.001 # pitch
        cov[35] = 0.005 # yaw
        msg.pose.covariance = cov

        self.pub_initialpose.publish(msg)

    def publish_rover_pose(self, t_map_base, stamp):
        """Publishes estimated rover pose for telemetry & visualization."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame

        msg.pose.position.x = float(t_map_base[0, 3])
        msg.pose.position.y = float(t_map_base[1, 3])
        msg.pose.position.z = 0.0

        r = R.from_matrix(t_map_base[:3, :3])
        q = r.as_quat()
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])

        self.pub_rover_pose.publish(msg)

    def lookup_transform(self, target_frame, source_frame, time):
        """Looks up 4x4 homogeneous transformation matrix between frames."""
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=0.05)
            )
            trans = [
                tf_stamped.transform.translation.x,
                tf_stamped.transform.translation.y,
                tf_stamped.transform.translation.z
            ]
            rot = [
                tf_stamped.transform.rotation.x,
                tf_stamped.transform.rotation.y,
                tf_stamped.transform.rotation.z,
                tf_stamped.transform.rotation.w
            ]
            mat = np.eye(4)
            mat[:3, :3] = R.from_quat(rot).as_matrix()
            mat[:3, 3] = trans
            return mat
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
