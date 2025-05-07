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

import _thread as thread
import threading
import time
import os
import rclpy
import socket
from rclpy.logging import get_logger


def cl_black(msge): return '\033[30m' + msge + '\033[0m'
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_orange(msge): return '\033[33m' + msge + '\033[0m'
def cl_blue(msge): return '\033[34m' + msge + '\033[0m'
def cl_purple(msge): return '\033[35m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'
def cl_lightgrey(msge): return '\033[37m' + msge + '\033[0m'
def cl_darkgrey(msge): return '\033[90m' + msge + '\033[0m'
def cl_lightred(msge): return '\033[91m' + msge + '\033[0m'
def cl_lightgreen(msge): return '\033[92m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_lightblue(msge): return '\033[94m' + msge + '\033[0m'
def cl_pink(msge): return '\033[95m' + msge + '\033[0m'
def cl_lightcyan(msge): return '\033[96m' + msge + '\033[0m'


class UDPSocket:
    def __init__(self,ip,port,node):
        # ROS2 logger
        self.logger = get_logger(node)
        self.BUFFER_SIZE = 4096
        self.isconnected = False
        self.node_name = node
        self.ip = ip
        self.port = port
        self.udp = None
        self.client_address = None
        self.connection_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.max_reconnection_attempts = 5
        self.reconnection_delay = 2  # seconds
        self.last_heartbeat = time.time()
        self.heartbeat_timeout = 15  # seconds - Extended to 15 seconds (3x the send rate)
        self.running = True
        self.startup_grace_period = 600  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        self.startup_time = time.time()
        
        # Add connection monitoring variables
        self.connection_status_publisher = None
        self.connection_status_timer = None
        self.connection_check_interval = 5.0  # seconds
        
        # Tracking for which ROS node is using this socket 
        self.ros_node = None
        
        # Track more detailed connection state
        self.connection_state = {
            'bound': False,              # Socket is bound to the port
            'connected': False,          # Remote client has connected
            'last_data_time': 0,         # Last time data was received
            'bytes_received': 0,         # Total bytes received
            'port': self.port,           # Port we're listening on
            'protocol': 'UDP'            # Protocol type
        }

        # Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.lbr_sensordata = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None


        threading.Thread(target=self.connect_to_socket).start()

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):
        """Connect to socket with retry mechanism"""
        attempt = 0
        
        while self.running:
            try:
                self.logger.info(f'Starting up node: {self.node_name}, IP: {self.ip}, Port: {self.port}')
                self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp.settimeout(600.0)  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
                self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)  # Increased buffer
                
                try:
                    self.udp.bind((self.ip, self.port))
                    self.connection_state['bound'] = True
                    self.logger.info(f"Successfully bound to port {self.port}")
                except socket.error as e:
                    self.logger.error(f"Error binding to port {self.port}: {e}")
                    self.connection_state['bound'] = False
                    raise e
                
                # Wait for initial data to establish connection
                connection_wait_start = time.time()
                while not self.isconnected and (time.time() - connection_wait_start) < 600:
                    try:
                        data, client_addr = self.udp.recvfrom(self.BUFFER_SIZE)
                        if data:  # If we get any data, we consider ourselves connected
                            with self.connection_lock:
                                self.client_address = client_addr
                                self.isconnected = True
                                self.last_heartbeat = time.time()
                            break
                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.logger.error(f"Error during initial connection: {e}")
                        break
                
                if not self.isconnected:
                    raise Exception("Timed out waiting for initial connection")
                    
                self.logger.info(f'Connected to client at {self.client_address}')
                # Send an initial hello message to confirm connection
                self.udp.sendto("hello KUKA".encode('utf-8'), self.client_address)
                attempt = 0  # Reset attempt counter on successful connection
                
                # Heartbeat sending thread (every 5 seconds as per protocol)
                def send_heartbeat():
                    while self.isconnected and self.running:
                        try:
                            if self.client_address:
                                self.udp.sendto("heartbeat".encode('utf-8'), self.client_address)
                        except:
                            pass
                        time.sleep(5)  # Send heartbeat every 5 seconds as specified in protocol
                
                # Start heartbeat thread
                thread.start_new_thread(send_heartbeat, ())
                
                # Main data processing loop
                while self.isconnected and self.running:
                    try:
                        data, addr = self.udp.recvfrom(self.BUFFER_SIZE)
                        if data:
                            self.last_heartbeat = time.time()  # Update heartbeat on successful receive
                            self.connection_state['last_data_time'] = time.time()
                            self.connection_state['bytes_received'] += len(data)
                            
                            # Log the size of incoming data for debugging
                            self.logger.debug(f"Received UDP data of size {len(data)} bytes from {self.node_name}")
                            
                            # Try to decode as UTF-8, but handle binary data gracefully
                            try:
                                data_str = data.decode('utf-8')
                                
                                # Process the received data
                                try:
                                    # If it's just a heartbeat message, don't process further
                                    if data_str.strip() in ["heartbeat", "ping", ""]:
                                        continue
                                        
                                    cmd_splt = data_str.split(">")[1].split() if ">" in data_str else data_str.split()
                                    
                                    if not cmd_splt:  # Skip empty commands
                                        continue
                                        
                                    with self.data_lock:
                                        if cmd_splt[0] == 'odometry':
                                            self.odometry = cmd_splt
                                        elif cmd_splt[0] == 'laserScan':
                                            if cmd_splt[2] == '1801':
                                                self.laserScanB1.append(cmd_splt)
                                            elif cmd_splt[2] == '1802':
                                                self.laserScanB4.append(cmd_splt)
                                        elif cmd_splt[0] == 'kmp_statusdata':
                                            self.kmp_statusdata = cmd_splt
                                        elif cmd_splt[0] == 'lbr_statusdata':
                                            self.lbr_statusdata = cmd_splt
                                        elif cmd_splt[0] == 'lbr_sensordata':
                                            self.lbr_sensordata.append(cmd_splt)
                                except Exception as e:
                                    self.logger.warn(f"Error processing command: {e}")
                                    # Don't break the connection for parsing errors
                            except UnicodeDecodeError:
                                # binary data => heartbeat skip
                                pass
                    except socket.timeout:
                        # Socket timeout is not an error, just continue
                        continue
                    except Exception as e:
                        self.logger.warn(f"Error receiving data: {e}")
                        with self.connection_lock:
                            self.isconnected = False
                        break
                        
            except Exception as e:
                attempt += 1
                err_type = type(e).__name__
                self.logger.error(f'Connection attempt {attempt} failed: {err_type}: {e}')
                # Add more specific debugging for common errors
                if isinstance(e, socket.error):
                    if e.errno == 111:  # Connection refused
                        self.logger.warn(f"  → The robot may not be listening on this port yet")
                    elif e.errno == 110:  # Connection timeout
                        self.logger.warn(f"  → Network route exists but robot not responding")
                if attempt >= self.max_reconnection_attempts:
                    self.logger.error(f'Maximum reconnection attempts reached. Waiting longer...')
                    time.sleep(self.reconnection_delay * 5)
                    attempt = 0  # Reset counter but keep trying
                else:
                    time.sleep(self.reconnection_delay)
                    
                try:
                    if self.udp:
                        self.udp.close()
                except:
                    pass
                
            # If we get here, connection has been lost, try to clean up
            self.logger.warn(f"Connection lost, cleaning up...")
            with self.connection_lock:
                try:
                    if self.udp:
                        self.udp.close()
                except Exception as ex:
                    self.logger.error(f"Error during cleanup: {ex}")
            
            if not self.running:
                break
                
            self.logger.info(f"Attempting to reconnect in {self.reconnection_delay} seconds...")
            time.sleep(self.reconnection_delay)

    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        try:
            thread.start_new_thread(self.__send, (cmd,))
        except:
            self.logger.error('Error: sending message thread failed')

    def __send(self, cmd):
        encoded_cmd = cmd.encode() # Encode to bytes
        self.udp.sendto(encoded_cmd, self.client_address)
