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

import threading
import time
import os
import rclpy
import socket
import logging


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


class TCPSocket:
    def __init__(self, ip, port, node):
        self.BUFFER_SIZE = 4000
        self.isconnected = False
        self.node_name = node
        self.ip = ip
        self.port = port
        self.tcp = None
        self.connection = None
        self.connection_lock = threading.Lock()
        self.max_reconnection_attempts = 5
        self.reconnection_delay = 2  # seconds
        self.last_heartbeat = time.time()
        self.heartbeat_timeout = 15  # seconds - Extended to 15 seconds (3x the send rate)
        self.running = True
        self.startup_grace_period = 600  # INCREASED TO 10 MINUTES FOR TROUBLESHOOTING
        self.startup_time = time.time()

        # Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None
        self.lbr_sensordata = []
        self.is_lbr_moving = False

        # Start the connection thread
        threading.Thread(target=self.connect_to_socket, daemon=True).start()
        # Start the heartbeat monitor
        threading.Thread(target=self.monitor_heartbeat, daemon=True).start()

    def close(self):
        self.running = False
        self.isconnected = False
        try:
            if self.connection:
                self.connection.shutdown(socket.SHUT_RDWR)
                self.connection.close()
            if self.tcp:
                self.tcp.close()
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
                self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_address = (self.ip, self.port)
                
                # Add socket options for better diagnostics
                self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                # Set TCP keepalive properties if available
                if hasattr(socket, 'TCP_KEEPIDLE'):
                    self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 30)
                if hasattr(socket, 'TCP_KEEPINTVL'):
                    self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
                if hasattr(socket, 'TCP_KEEPCNT'):
                    self.tcp.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
                
                print(cl_cyan(f'Binding to {server_address}...'))
                self.tcp.bind(server_address)
                self.tcp.listen(10)
                self.tcp.settimeout(600)
                
                print(cl_green(f'Successfully bound to {server_address}, waiting for connection...'))
                
                with self.connection_lock:
                    print(cl_cyan(f'Waiting for client to connect on port {self.port}...'))
                    self.connection, client_address = self.tcp.accept()
                    print(cl_green(f'Client connected from {client_address}'))
                    
                    # Connection settings for better data flow
                    self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.connection.settimeout(600)
                    self.isconnected = True
                    self.last_heartbeat = time.time()
                
                # Add heartbeat sending thread
                def send_heartbeat():
                    last_heartbeat_send = 0
                    while self.isconnected and self.running:
                        try:
                            current_time = time.time()
                            # Only send a heartbeat every 5 seconds to avoid flooding
                            if current_time - last_heartbeat_send >= 5:
                                with self.connection_lock:
                                    if self.isconnected and self.connection:
                                        # Format according to protocol: 10-digit length prefix + "heartbeat"
                                        msg = "heartbeat"
                                        length = str(len(msg)).zfill(10)  # 10-digit length prefix
                                        heartbeat_msg = length + msg
                                        self.connection.sendall(heartbeat_msg.encode("UTF-8"))
                                        last_heartbeat_send = current_time
                        except Exception as e:
                            print(cl_yellow(f"Heartbeat send error: {e}"))
                        time.sleep(1)  # Check every second but only send every 5
                
                # Start heartbeat thread
                threading.Thread(target=send_heartbeat, daemon=True).start()

                # Main data processing loop
                while self.isconnected and self.running:
                    try:
                        data = self.connection.recv(self.BUFFER_SIZE)
                        if data:
                            self.last_heartbeat = time.time()
                            data_len = len(data)
                            print(cl_green(f"Received {data_len} bytes of data from client"))
                            
                            # Try to decode and log it for debugging
                            try:
                                data_str = data.decode('utf-8').strip()
                                if data_len < 200:  # Only print short messages to avoid flooding
                                    print(cl_green(f"Data content: {data_str}"))
                                else:
                                    print(cl_green(f"Data starts with: {data_str[:50]}..."))
                                    
                                if data_str in ["heartbeat", "ping", ""]:
                                    print(cl_cyan("Received heartbeat message"))
                                    continue
                                
                                # Process the received command
                                for pack in data_str.split(">"):
                                    cmd_splt = pack.split()
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
                                
                            except UnicodeDecodeError:
                                print(cl_yellow(f"Received binary data of {data_len} bytes"))
                        else:
                            print(cl_yellow(f"Received empty data, client may have disconnected"))
                            with self.connection_lock:
                                self.isconnected = False
                            break
                            
                    except socket.timeout:
                        continue
                    except Exception as e:
                        print(cl_yellow(f"Error receiving data: {type(e).__name__}: {e}"))
                        continue
                        
            except socket.error as e:
                err_type = type(e).__name__
                print(cl_yellow(f'Connection error: {err_type}: {e}'))
                attempt += 1
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
                    if self.tcp:
                        self.tcp.close()
                except:
                    pass
                
                continue

    def send(self, cmd):
        """Thread-safe send command with verification"""
        if not self.isconnected or not self.connection:
            print(cl_red(f"Cannot send command, not connected: {cmd}"))
            return False
            
        try:
            with self.connection_lock:
                if self.isconnected and self.connection:
                    # Special highlighting for movement commands
                    is_movement = "cmd_vel" in cmd
                    prefix = cl_green("MOVEMENT:") if is_movement else cl_cyan("SENDING:")
                    print(f"{prefix} {cmd}")
                    
                    # Format for the protocol: 10-digit length prefix + content
                    length = str(len(cmd)).zfill(10)  # 10-digit length prefix
                    message = length + cmd
                    
                    # Log exact byte representation for debugging
                    message_bytes = message.encode('utf-8')
                    print(f"Raw bytes [{len(message_bytes)}]: {message_bytes}")
                    
                    # Actually send the data
                    self.connection.sendall(message_bytes)
                    return True
                return False
        except Exception as e:
            print(cl_red(f'Error sending message: {e}'))
            with self.connection_lock:
                self.isconnected = False
            return False

    def recvmsg(self):
        header_len = 10
        msglength = 0

        byt_len = b""  # Initialize as bytes, not string
        byt_len = self.connection.recv(header_len)
        diff_header = header_len - len(byt_len)
        while diff_header > 0:
            # Use bytes concatenation instead of extend
            byt_len = byt_len + self.connection.recv(diff_header)
            diff_header = header_len - len(byt_len)

        msglength = int(byt_len.decode("utf-8")) + 1  # include crocodile and space
        msg = b""  # Initialize as bytes

        if msglength > 0 and msglength < 5000:
            msg = self.connection.recv(msglength)
            diff_msg = msglength - len(msg)
            while diff_msg > 0:
                newmsg = self.connection.recv(diff_msg)
                # Use bytes concatenation instead of extend
                msg = msg + newmsg
                diff_msg = msglength - len(msg)
        return msg
