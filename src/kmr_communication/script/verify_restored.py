#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class VerificationTest(Node):
    def __init__(self):
        super().__init__('verification_test')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Verification test node initialized")
        
    def run_test(self):
        """Run a simple movement test sequence"""
        # Wait for connections to establish
        self.get_logger().info("Waiting 3 seconds for connections to establish...")
        time.sleep(3.0)
        
        # Forward
        msg = Twist()
        msg.linear.x = 0.1
        self.get_logger().info("Sending FORWARD command (0.1 m/s)")
        self.publisher_.publish(msg)
        time.sleep(2.0)
        
        # Stop
        msg = Twist()
        self.get_logger().info("Sending STOP command")
        self.publisher_.publish(msg)
        time.sleep(1.0)
        
        # Rotate
        msg = Twist()
        msg.angular.z = 0.1
        self.get_logger().info("Sending ROTATE command (0.1 rad/s)")
        self.publisher_.publish(msg)
        time.sleep(2.0)
        
        # Stop again
        msg = Twist()
        self.get_logger().info("Sending final STOP command")
        self.publisher_.publish(msg)
        
        self.get_logger().info("Test complete - check if robot moved correctly")

def main():
    rclpy.init()
    tester = VerificationTest()
    
    try:
        tester.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
