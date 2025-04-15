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
        self.BUFFER_SIZE = 65535  # Updated according to protocol - increased from 4096
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
        self.heartbeat_timeout = 10  # seconds
        self.running = True
        self.startup_grace_period = 30  # Give the robot 30 seconds to start up before reporting issues
        self.startup_time = time.time()

        #Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.lbr_sensordata = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None
        self.is_lbr_moving = False

        threading.Thread(target=self.connect_to_socket, daemon=True).start()
        threading.Thread(target=self.monitor_heartbeat, daemon=True).start()

    def close(self):
        self.running = False
        self.isconnected = False
        try:
            if self.udp:
                self.udp.close()
        except Exception as e:
            print(cl_red(f"Error closing socket: {e}"))

    def monitor_heartbeat(self):
        """Monitor connection health and attempt reconnection if needed"""
        last_reported_status = None  # None means no status has been reported yet
        initialization_grace_period = self.startup_grace_period  # Give the robot this many seconds to connect before reporting status
        start_time = time.time()
        
        while self.running:
            # During startup grace period, don't report down status
            if time.time() - self.startup_time < initialization_grace_period:
                time.sleep(1)
                continue
                
            # First time status check after grace period
            if last_reported_status is None:
                last_reported_status = self.isconnected
                # Only report if the connection is up after grace period
                if self.isconnected:
                    print(cl_green(f"Connection established for {self.node_name}"))
                else:
                    print(cl_yellow(f"Unable to establish connection for {self.node_name}. Will keep trying in background."))
                    
            # Check if we're connected before checking heartbeat
            elif self.isconnected:
                if (time.time() - self.last_heartbeat) > self.heartbeat_timeout:
                    print(cl_yellow(f"Heartbeat timeout for {self.node_name}. Attempting to reconnect..."))
                    
                    # Only report once when status changes
                    if last_reported_status:
                        print(cl_yellow(f"Connection lost for {self.node_name}"))
                        last_reported_status = False
                    
                    # Mark as disconnected but let the main loop handle cleanup
                    with self.connection_lock:
                        self.isconnected = False
                        try:
                            if self.udp:
                                self.udp.close()
                        except:
                            pass
            else:
                # If we're not connected but were previously, report once
                if last_reported_status:
                    print(cl_yellow(f"Connection down for {self.node_name}"))
                    last_reported_status = False
                    
            # If we reconnected, update status tracking
            if not last_reported_status and self.isconnected:
                print(cl_green(f"Connection restored for {self.node_name}"))
                last_reported_status = True
                
            time.sleep(1)

    def connect_to_socket(self):
        """Connect to socket with retry mechanism"""
        attempt = 0
        
        while self.running:
            try:
                print(cl_cyan(f'Starting up node: {self.node_name}, IP: {self.ip}, Port: {self.port}'))
                self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp.settimeout(1.0)  # Non-blocking as per protocol
                self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)  # Increased buffer
                self.udp.bind((self.ip, self.port))
                
                # Wait for initial data to establish connection
                connection_wait_start = time.time()
                while not self.isconnected and (time.time() - connection_wait_start) < 30:  # 30 sec connection timeout
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
                        print(cl_red(f"Error during initial connection: {e}"))
                        break
                
                if not self.isconnected:
                    raise Exception("Timed out waiting for initial connection")
                    
                print(cl_green(f'Connected to client at {self.client_address}'))
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
                            
                            # Log the size of incoming data for debugging
                            print(f"Received UDP data of size {len(data)} bytes from {self.node_name}")
                            
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
                                    print(cl_yellow(f"Error processing command: {e}"))
                                    # Don't break the connection for parsing errors
                            except UnicodeDecodeError:
                                # This is binary data, just use it as a heartbeat
                                pass
                    except socket.timeout:
                        # Socket timeout is not an error, just continue
                        continue
                    except Exception as e:
                        print(cl_yellow(f"Error receiving data: {e}"))
                        with self.connection_lock:
                            self.isconnected = False
                        break
                        
            except Exception as e:
                attempt += 1
                err_type = type(e).__name__
                print(cl_red(f'Connection attempt {attempt} failed: {err_type}: {e}'))
                # Add more specific debugging for common errors
                if isinstance(e, socket.error):
                    if e.errno == 111:  # Connection refused
                        print(cl_yellow(f"  → The robot may not be listening on this port yet"))
                    elif e.errno == 110:  # Connection timeout
                        print(cl_yellow(f"  → Network route exists but robot not responding"))
                if attempt >= self.max_reconnection_attempts:
                    print(cl_red(f'Maximum reconnection attempts reached. Waiting longer...'))
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
            print(cl_yellow(f"Connection lost, cleaning up..."))
            with self.connection_lock:
                try:
                    if self.udp:
                        self.udp.close()
                except Exception as ex:
                    print(cl_red(f"Error during cleanup: {ex}"))
            
            if not self.running:
                break
                
            print(cl_yellow(f"Attempting to reconnect in {self.reconnection_delay} seconds..."))
            time.sleep(self.reconnection_delay)

    # Each send command runs as a thread. May need to control the maximum running time (valid time to send a command).
    def send(self, cmd):
        """Thread-safe send command"""
        if not self.isconnected or not self.client_address:
            print(cl_yellow(f"Cannot send command, not connected"))
            return False
            
        try:
            with self.connection_lock:
                if self.isconnected and self.udp and self.client_address:
                    encoded_cmd = cmd.encode() 
                    self.udp.sendto(encoded_cmd, self.client_address)
                    return True
                return False
        except Exception as e:
            print(cl_red(f'Error sending message: {e}'))
            with self.connection_lock:
                self.isconnected = False
            return False
