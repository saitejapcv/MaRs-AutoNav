#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus  # <--- NEW IMPORT

class ConeHunter(Node):
    def __init__(self):
        super().__init__('cone_hunter')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y):
        self.get_logger().info(f"Setting target coordinate: X={x}, Y={y}")
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0 
        
        self.get_logger().info("Sending goal to Nav2...")
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal instantly!')
            return
        
        self.get_logger().info('Goal accepted by Nav2! Rover is driving...')
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # NEW LOGIC: Actually check if Nav2 succeeded or aborted!
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ SUCCESS: Rover has reached the target coordinate!')
        else:
            self.get_logger().error(f'❌ FAILED: Nav2 aborted the mission. (The path is blocked!)')

def main(args=None):
    rclpy.init(args=args)
    hunter = ConeHunter()
    
    hunter.send_goal(4.0, 0.0) 
    
    rclpy.spin(hunter)
    rclpy.shutdown()

if __name__ == '__main__':
    main()