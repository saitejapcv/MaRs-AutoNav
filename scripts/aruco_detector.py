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

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # This dictionary is our "Sticky Memory" for the Noise Filter
        self.stable_markers = {} 
        
        # Camera Math (Focal length and center for your 800x800 camera with an 80-degree FOV)
        self.camera_matrix = np.array([
            [476.7, 0.0, 400.0],
            [0.0, 476.7, 400.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        self.get_logger().info("Visual Cortex Online: Scanning for ArUco Markers...")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 1. Detect the Markers
        try:
            dictionary = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejected = aruco.detectMarkers(gray, dictionary, parameters=parameters)
        except AttributeError:
            # Fallback just in case OpenCV updates its syntax in the background
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters()
            detector = aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Draw boxes around what it sees
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # 2. Extract Pose (Assuming the marker is 20 centimeters wide)
            marker_size = 0.2 
            try:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    x, y, z = tvecs[i][0]
                    
                    # 3. NOISE FILTER (Preventing Jitter)
                    if marker_id in self.stable_markers:
                        old_x, old_y, old_z = self.stable_markers[marker_id]
                        
                        # Calculate exactly how far the marker appeared to move (Euclidean Distance)
                        drift = math.sqrt((x-old_x)**2 + (y-old_y)**2 + (z-old_z)**2)
                        
                        if drift < 0.05: 
                            # If it moved less than 5cm, it's just camera jitter! Use the old stable math.
                            x, y, z = old_x, old_y, old_z 
                        else:
                            # It actually moved, update the memory
                            self.stable_markers[marker_id] = [x, y, z] 
                    else:
                        # First time seeing this specific marker ID
                        self.stable_markers[marker_id] = [x, y, z] 

                    # 4. Publish the clean locations to the ROS 2 Transform System
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_optical_link'
                    t.child_frame_id = f'aruco_marker_{marker_id}'
                    
                    t.transform.translation.x = float(x)
                    t.transform.translation.y = float(y)
                    t.transform.translation.z = float(z)
                    t.transform.rotation.w = 1.0 # Keeping rotation flat for simplicity
                    
                    self.tf_broadcaster.sendTransform(t)
                    
                    # Draw text on screen so you can see the math working
                    cv2.putText(cv_image, f"ID: {marker_id} Z:{z:.2f}m", 
                                (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            except AttributeError:
                pass 

        cv2.imshow("Rover Camera View (OpenCV)", cv_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'): # Press 'c' to capture
            filename = f"cone_data_{self.get_clock().now().nanoseconds}.jpg"
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Captured: {filename}")
            
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
