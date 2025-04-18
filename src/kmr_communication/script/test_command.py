#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class CommandTester(Node):
    def __init__(self):
        super().__init__('command_tester')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.max_count = 10
        self.get_logger().info("Command tester started. Will publish 10 test commands...")
        
    def timer_callback(self):
        msg = Twist()
        
        # Alternate between forward movement and rotation
        if self.count % 2 == 0:
            msg.linear.x = 0.1  # Forward at 0.1 m/s
            command_type = "FORWARD"
        else:
            msg.angular.z = 0.2  # Rotate at 0.2 rad/s
            command_type = "ROTATE"
            
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published {command_type} command: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        
        self.count += 1
        if self.count >= self.max_count:
            self.get_logger().info("Test complete. Shutting down...")
            self.timer.cancel()
            time.sleep(1)  # Give time for the last message to be processed
            rclpy.shutdown()

def main():
    rclpy.init()
    tester = CommandTester()
    rclpy.spin(tester)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
