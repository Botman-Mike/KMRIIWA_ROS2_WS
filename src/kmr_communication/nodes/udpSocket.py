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
        self.heartbeat_timeout = 10  # seconds
        self.running = True

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
        while self.running:
            if self.isconnected and (time.time() - self.last_heartbeat) > self.heartbeat_timeout:
                print(cl_yellow(f"Heartbeat timeout for {self.node_name}. Attempting to reconnect..."))
                with self.connection_lock:
                    self.isconnected = False
                    try:
                        if self.udp:
                            self.udp.close()
                    except:
                        pass
            time.sleep(1)

    def connect_to_socket(self):
        """Connect to socket with retry mechanism"""
        attempt = 0
        
        while self.running:
            try:
                print(cl_cyan(f'Starting up node: {self.node_name}, IP: {self.ip}, Port: {self.port}'))
                self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.udp.settimeout(1.0)  # Increased timeout for better stability
                self.udp.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
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
                
                # Main data processing loop
                while self.isconnected and self.running:
                    try:
                        data, addr = self.udp.recvfrom(self.BUFFER_SIZE)
                        if data:
                            self.last_heartbeat = time.time()  # Update heartbeat on successful receive
                            data_str = data.decode('utf-8')
                            
                            # Process the received data
                            try:
                                cmd_splt = data_str.split(">")[1].split() if ">" in data_str else data_str.split()
                                
                                with self.data_lock:
                                    if len(cmd_splt) and cmd_splt[0] == 'odometry':
                                        self.odometry = cmd_splt
                                    if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                                        if cmd_splt[2] == '1801':
                                            self.laserScanB1.append(cmd_splt)
                                        elif cmd_splt[2] == '1802':
                                            self.laserScanB4.append(cmd_splt)
                                    if len(cmd_splt) and cmd_splt[0] == 'kmp_statusdata':
                                        self.kmp_statusdata = cmd_splt
                                    if len(cmd_splt) and cmd_splt[0] == 'lbr_statusdata':
                                        self.lbr_statusdata = cmd_splt
                                    if len(cmd_splt) and cmd_splt[0] == 'lbr_sensordata':
                                        self.lbr_sensordata.append(cmd_splt)
                            except Exception as e:
                                print(cl_yellow(f"Error processing data: {e}"))
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
                print(cl_red(f'Connection attempt {attempt} failed: {e}'))
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
