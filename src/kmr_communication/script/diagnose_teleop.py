#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import sys
import subprocess

class TeleopDiagnostic(Node):
    def __init__(self):
        super().__init__('teleop_diagnostic')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.test_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_msg_time = None
        self.msg_count = 0
        
        # Create a timer to check if we've received messages
        self.timer = self.create_timer(2.0, self.check_messages)
        self.get_logger().info("Teleop diagnostic node started - monitoring /cmd_vel topic")
        self.get_logger().info("Will send test movement commands in 5 seconds if no messages detected")
        
    def cmd_vel_callback(self, msg):
        now = self.get_clock().now()
        self.msg_count += 1
        self.last_msg_time = now
        
        # Print received commands
        self.get_logger().info(f"Received command #{self.msg_count}: "
                               f"linear=({msg.linear.x:.4f}, {msg.linear.y:.4f}, {msg.linear.z:.4f}) "
                               f"angular=({msg.angular.x:.4f}, {msg.angular.y:.4f}, {msg.angular.z:.4f})")
    
    def check_messages(self):
        if self.last_msg_time is None:
            self.get_logger().warn("No /cmd_vel messages received yet!")
            
            # After 5 seconds with no messages, send test commands
            if self.msg_count == 0 and self.get_clock().now().to_msg().sec > 5:
                self.send_test_commands()
        else:
            # Check if it's been a while since the last message
            now = self.get_clock().now()
            diff = now - self.last_msg_time
            if diff.nanoseconds / 1e9 > 5.0:  # More than 5 seconds
                self.get_logger().warn(f"No /cmd_vel messages received for {diff.nanoseconds/1e9:.1f} seconds!")
    
    def check_topic_publishers(self):
        self.get_logger().info("Checking topic publishers...")
        
        try:
            # Run ros2 topic info command
            result = subprocess.run(['ros2', 'topic', 'info', '/cmd_vel'], 
                                   capture_output=True, text=True)
            self.get_logger().info(f"Topic info:\n{result.stdout}")
            
            # Check nodes
            result = subprocess.run(['ros2', 'node', 'list'], 
                                   capture_output=True, text=True)
            self.get_logger().info(f"Active nodes:\n{result.stdout}")
        except Exception as e:
            self.get_logger().error(f"Error checking topics: {e}")
    
    def send_test_commands(self):
        """Send test movement commands"""
        self.check_topic_publishers()
        self.get_logger().info("Sending test commands to /cmd_vel...")
        
        # Forward command
        msg = Twist()
        msg.linear.x = 0.1
        self.get_logger().info("TEST: Sending FORWARD command (0.1 m/s)")
        self.test_publisher.publish(msg)
        time.sleep(1.0)
        
        # Rotation command
        msg = Twist()
        msg.angular.z = 0.2
        self.get_logger().info("TEST: Sending ROTATE command (0.2 rad/s)")
        self.test_publisher.publish(msg)
        time.sleep(1.0)
        
        # Stop command
        msg = Twist()
        self.get_logger().info("TEST: Sending STOP command")
        self.test_publisher.publish(msg)
        
        self.get_logger().info("Test commands sent. Check if they were received by the command node.")

def main():
    rclpy.init()
    node = TeleopDiagnostic()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
