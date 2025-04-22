#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys

class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Movement test node initialized")
        
    def send_forward(self, speed=0.1, duration=1.0):
        """Send a forward movement command"""
        msg = Twist()
        msg.linear.x = speed
        self.get_logger().info(f"Sending forward command: {speed} m/s")
        self.publisher.publish(msg)
        time.sleep(duration)
        
    def send_rotate(self, speed=0.2, duration=1.0):
        """Send a rotation command"""
        msg = Twist()
        msg.angular.z = speed
        self.get_logger().info(f"Sending rotation command: {speed} rad/s")
        self.publisher.publish(msg)
        time.sleep(duration)
        
    def send_stop(self):
        """Send stop command"""
        msg = Twist()
        self.get_logger().info("Sending STOP command")
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = MovementTester()
    
    # Test sequence
    try:
        node.get_logger().info("Starting movement test...")
        
        # Forward
        node.send_forward(0.15, 2.0)
        node.send_stop()
        time.sleep(1.0)
        
        # Rotate
        node.send_rotate(0.25, 2.0)
        node.send_stop()
        time.sleep(1.0)
        
        # Combined
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.1
        node.get_logger().info("Sending combined movement")
        node.publisher.publish(twist)
        time.sleep(2.0)
        
        node.send_stop()
        node.get_logger().info("Test complete")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
