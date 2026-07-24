#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ultralytics import YOLO

class MasterVisionNode(Node):
    def __init__(self):
        super().__init__('master_vision_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # --- 1. YOLO Setup ---
        model_path = '/home/yashvant/ros2_ws/src/ares_nova/cone_model.pt'
        self.model = YOLO(model_path)
        
        # --- 2. ArUco Setup ---
        self.stable_markers = {} 
        self.camera_matrix = np.array([
            [476.7, 0.0, 400.0],
            [0.0, 476.7, 400.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        self.get_logger().info("Unified Visual Cortex Online: YOLO + ArUco Active!")

    def image_callback(self, msg):
        # Convert ROS 2 Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # ==========================================
        # PHASE 1: YOLO NEURAL NETWORK
        # ==========================================
        results = self.model(cv_image, verbose=False)
        # This creates a frame with the cone boxes drawn on it
        annotated_frame = results[0].plot() 
        
        # ==========================================
        # PHASE 2: ARUCO GEOMETRY
        # ==========================================
        try:
            dictionary = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejected = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        except AttributeError:
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Draw ArUco boxes ON TOP of the YOLO frame
            aruco.drawDetectedMarkers(annotated_frame, corners, ids)
            marker_size = 0.2 
            
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    x, y, z = tvecs[i][0]
                    
                    # Noise Filter
                    if marker_id in self.stable_markers:
                        old_x, old_y, old_z = self.stable_markers[marker_id]
                        drift = math.sqrt((x-old_x)**2 + (y-old_y)**2 + (z-old_z)**2)
                        
                        if drift < 0.05: 
                            x, y, z = old_x, old_y, old_z 
                        else:
                            self.stable_markers[marker_id] = [x, y, z] 
                    else:
                        self.stable_markers[marker_id] = [x, y, z] 

                    # Publish TF
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_optical_link'
                    t.child_frame_id = f'aruco_marker_{marker_id}'
                    t.transform.translation.x = float(x)
                    t.transform.translation.y = float(y)
                    t.transform.translation.z = float(z)
                    t.transform.rotation.w = 1.0 
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Draw Distance Math
                    cv2.putText(annotated_frame, f"ID: {marker_id} Z:{z:.2f}m", 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            except AttributeError:
                pass 

        # Display the final fused frame
        cv2.imshow("Ares-Nova Unified Vision", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MasterVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

