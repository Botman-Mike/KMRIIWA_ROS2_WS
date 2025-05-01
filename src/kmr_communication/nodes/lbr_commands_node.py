#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kmr_msgs.msg import LbrStatusdata
from kmr_msgs.action import MoveManipulator
from tcpSocket import TCPSocket
from udpSocket import UDPSocket
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor



def cl_red(msge): return '\033[31m' + msge + '\033[0m'



class LbrCommandsNode(Node):
    def __init__(self,connection_type,robot):
        super().__init__('lbr_commands_node')
        self.name = 'lbr_commands_node'
        self.declare_parameter('port', 30005)
        self.declare_parameter('reconnect_interval', 5.0)  # Seconds between reconnection attempts
        self.declare_parameter('command_timeout', 600.0)  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        self.declare_parameter('respect_safety', True)  # Whether to respect safety signals
        
        port = int(self.get_parameter('port').value)
        if robot == 'KMR1':
            self.declare_parameter('KMR1/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR1/ip').value)
        elif robot == 'KMR2':
            self.declare_parameter('KMR2/ip', '172.31.1.206')
            ip = str(self.get_parameter('KMR2/ip').value)
        else:
            ip = None

        if connection_type == 'TCP':
            self.soc = TCPSocket(ip,port,self.name)
            # Register socket close on shutdown, with fallback
            try:
                self.add_on_shutdown_callback(self.soc.close)
            except AttributeError:
                rclpy.get_default_context().on_shutdown(self.soc.close)
        elif connection_type == 'UDP':
            self.soc = UDPSocket(ip,port,self.name)
            # Register socket close on shutdown, with fallback
            try:
                self.add_on_shutdown_callback(self.soc.close)
            except AttributeError:
                rclpy.get_default_context().on_shutdown(self.soc.close)
        else:
            self.soc = None


        self.callback_group = ReentrantCallbackGroup()
        

        # Make a listener for relevant topics
        sub_manipulator_vel = self.create_subscription(String, 'manipulator_vel', self.manipulatorVel_callback, 10)
        sub_shutdown = self.create_subscription(String, 'shutdown', self.shutdown_callback, 10)
        sub_statusdata=self.create_subscription(LbrStatusdata, 'lbr_statusdata', self.status_callback, 10,callback_group=self.callback_group)
        self.path_server = ActionServer(self,MoveManipulator,'move_manipulator', self.move_manipulator_callback,callback_group=self.callback_group)

        self.point_publisher = self.create_publisher(Float64, 'vinkel', 20)

        self.done_moving = False
        self.last_path_variable = False
        self.command_queue_lock = threading.Lock()
        self.command_queue = []  # Queue for commands to ensure they're sent in order
        
        # Track robot safety status
        self.safe_to_move = True
        self.emergency_stop_active = False
        self.movement_in_progress = False
        
        # Create a worker thread to process command queue
        self.command_worker_thread = threading.Thread(target=self.process_command_queue, daemon=True)
        self.command_worker_thread.start()

        # Wait for initial connection
        connection_timeout = 600.0  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        start_time = time.time()
        
        self.get_logger().info('Waiting for initial connection...')
        while not self.soc.isconnected and time.time() - start_time < connection_timeout:
            time.sleep(0.1)
            
        if self.soc.isconnected:
            self.get_logger().info('Connected to manipulator successfully')
        else:
            self.get_logger().warn('Initial connection timed out, will keep trying in background')

    def connection_check_callback(self):
        """Periodically check connection status and publish it"""
        status_msg = Bool()
        status_msg.data = self.soc.isconnected
        self.connection_status_pub.publish(status_msg)
        
        # Log connection status periodically at debug level
        if self.soc.isconnected:
            self.get_logger().debug('Manipulator connection is active')
        else:
            self.get_logger().warn('Manipulator connection is down')

    def process_command_queue(self):
        """Worker thread to process command queue"""
        while rclpy.ok():
            if self.soc.isconnected and self.command_queue:
                with self.command_queue_lock:
                    if self.command_queue:
                        cmd = self.command_queue.pop(0)
                        if not self.soc.send(cmd):
                            self.get_logger().error(f"Failed to send command: {cmd}")
                            # Option to requeue failed commands
                            # self.command_queue.insert(0, cmd)
            time.sleep(0.01)  # Small sleep to prevent CPU hogging

    def queue_command(self, cmd):
        """Add command to queue for sending"""
        with self.command_queue_lock:
            self.command_queue.append(cmd)
        return True

    def safety_callback(self, msg):
        """Process safety status updates"""
        was_safe = self.safe_to_move
        self.safe_to_move = msg.data
        
        if was_safe and not self.safe_to_move:
            self.get_logger().warn("LBR has entered unsafe state - commands will be blocked")
            # Clear command queue when entering unsafe state
            with self.command_queue_lock:
                self.command_queue.clear()
            
            # If a movement is in progress, send a stop command
            if self.movement_in_progress:
                self.soc.send("stopLBRmotion")
                self.get_logger().warn("Stopping in-progress movement due to safety condition")
                
        elif not was_safe and self.safe_to_move:
            self.get_logger().info("LBR has returned to safe state - commands will be accepted")

    def estop_callback(self, msg):
        """Process emergency stop status"""
        was_stopped = self.emergency_stop_active
        self.emergency_stop_active = msg.data
        
        if not was_stopped and self.emergency_stop_active:
            self.get_logger().error("Emergency stop activated - all commands blocked")
            # Clear command queue when e-stop activated
            with self.command_queue_lock:
                self.command_queue.clear()
                
            # If a movement is in progress, send a stop command
            if self.movement_in_progress:
                self.soc.send("stopLBRmotion")
                self.get_logger().error("Emergency stopping in-progress movement")
                
        elif was_stopped and not self.emergency_stop_active:
            self.get_logger().info("Emergency stop cleared")

    def reset_callback(self, msg):
        """Handle safety reset requests"""
        if msg.data and self.soc.isconnected:
            self.soc.send("resetLBRSafetyStop")
            self.get_logger().info("Safety stop reset command sent to manipulator")

    def status_callback(self, data):
        if self.last_path_variable == False and data.path_finished == True:
            self.done_moving = True
            print("done_moving set to True")
        self.last_path_variable = data.path_finished


    def shutdown_callback(self, data):
        print(data)
        msg = "shutdown"
        self.soc.send(msg)
        self.soc.isconnected = False

    def manipulatorVel_callback(self, data):
        msg = 'setLBRmotion ' + data.data
        self.soc.send(msg)

    def move_manipulator_callback(self, goal_handle):
        self.path_callback(goal_handle.request.path)
        self.done_moving = False
        self.movement_in_progress = True
        
        # Wait for movement to complete with timeout
        timeout = 600.0  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        start_time = time.time()
        
        while not self.done_moving and time.time() - start_time < timeout:
            # Check if we should abort due to safety condition change during execution
            if (not self.soc.isconnected or 
                (self.respect_safety and not self.safe_to_move) or 
                self.emergency_stop_active):
                
                self.get_logger().error("Movement aborted due to safety condition change")
                goal_handle.abort()
                result = MoveManipulator.Result()
                result.success = False
                return result
                
            time.sleep(0.1)
            
        if self.done_moving:
            self.get_logger().info("Movement completed successfully")
            result = MoveManipulator.Result()
            result.success = True
            goal_handle.succeed()
            return result
        else:
            self.get_logger().error("Movement timed out")
            result = MoveManipulator.Result()
            result.success = False
            goal_handle.abort()
            return result

    def cancel_callback(self, goal_handle):
        """Handle cancellation of movement action"""
        self.get_logger().info("Cancelling movement")
        
        # Send a stop command to the manipulator
        if self.soc.isconnected:
            self.soc.send("stopLBRmotion")
            
        # Clear the command queue
        with self.command_queue_lock:
            self.command_queue.clear()
            
        return CancelResponse.ACCEPT

    def path_callback(self, data):
        i=1
        for point in data.points:
            positions = " ".join([str(s) for s in point.positions])
            velocities = " ".join([str(s) for s in point.velocities])
            accelerations = " ".join([str(s) for s in point.accelerations])
            x = Float64()
            x.data = point.positions[6]
            self.point_publisher.publish(x)
            if i == 1:
                type = "StartPoint"
            elif i == len(data.points):
                type = "EndPoint"
            else:
                type = "WayPoint"
            msg = 'pathPointLBR ' + ">" + type + ">" + positions + ">" + velocities + ">" + accelerations

            self.soc.send(msg)
            i+=1


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-c', '--connection')
    parser.add_argument('-ro', '--robot')
    args = parser.parse_args(remove_ros_args(args=argv))

    rclpy.init(args=argv)
    lbr_commands_node = LbrCommandsNode(args.connection,args.robot)

    executor = MultiThreadedExecutor()
    rclpy.spin(lbr_commands_node, executor)
    # Ensure socket closed before shutdown
    try:
        lbr_commands_node.soc.close()
    except Exception:
        pass

    try:
        lbr_commands_node.destroy_node()
        rclpy.shutdown()
    except:
        print(cl_red('Error: ') + "rclpy shutdown failed")


if __name__ == '__main__':
    main()
