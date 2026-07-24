#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOVisionNode(Node):
    def __init__(self):
        super().__init__('yolo_vision_node')
        
        # 1. Subscribe to the Gazebo camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.bridge = CvBridge()
        
        # 2. Load the custom AI brain we just trained!
        model_path = '/home/yashvant/ros2_ws/src/ares_nova/cone_model.pt'
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("Visual Cortex Online. Awaiting camera feed...")

    def image_callback(self, msg):
        # Convert ROS 2 Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Pass the image to the AI (verbose=False stops it from spamming the terminal)
        results = self.model(cv_image, verbose=False)
        
        # Tell YOLO to draw the bounding boxes and labels onto the image
        annotated_frame = results[0].plot()
        
        # Display the live AI feed
        cv2.imshow("Ares-Nova YOLO Vision", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
