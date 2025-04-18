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
    def __init__(self, ip, port,node):
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

        #Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None
        self.lbr_sensordata = []

        threading.Thread(target=self.connect_to_socket).start()

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):
        """Connect to socket with retry mechanism"""
        attempt = 0

        while self.running:
            try:
                print(cl_cyan(f'Starting up node: {self.node_name}, IP: {self.ip}, Port: {self.port}'))
                self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_address = (self.ip, self.port)
                
                # Add socket reuse option to prevent "Address already in use" errors
                self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                
                self.tcp.bind(server_address)
                
                # Increase backlog queue from default 3 to 10 to handle multiple connection attempts
                self.tcp.listen(10)
                
                # Extend timeout for initial connection - 10 minutes as requested
                self.tcp.settimeout(600)  
                
                print(cl_green(f'Socket bound and listening on {server_address}'))
                
                with self.connection_lock:
                    self.connection, client_address = self.tcp.accept()
                    
                    # Increase socket buffer sizes for performance
                    self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
                    self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)
                    
                    # Disable Nagle's algorithm for lower latency
                    self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    
                    # Use keep-alive to detect connection loss
                    self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    
                    # TCP keepalive on Linux - more aggressive to detect problems faster
                    if hasattr(socket, 'TCP_KEEPIDLE'):
                        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
                    if hasattr(socket, 'TCP_KEEPINTVL'):
                        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
                    if hasattr(socket, 'TCP_KEEPCNT'):
                        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)
                        
                    # Set reasonable timeout for socket operations
                    self.connection.settimeout(600)  # 10 minute operations timeout
                    self.isconnected = True
                    self.last_heartbeat = time.time()

                print(cl_green(f'Connected to client at {client_address}'))
                attempt = 0  # Reset attempt counter on successful connection
                time.sleep(1)

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
                            
                            # Try to decode as UTF-8, but handle binary data gracefully
                            try:
                                data_str = data.decode('utf-8').strip()
                                
                                # Check if this is a heartbeat-only message
                                if data_str in ["heartbeat", "ping", ""]:
                                    continue  # Skip processing for heartbeat messages
                                
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
                                # If it can't be decoded as text, just use it as a heartbeat
                                pass
                        else:
                            # Empty data means client disconnected
                            print(cl_yellow(f"Client disconnected - empty data received"))
                            with self.connection_lock:
                                self.isconnected = False
                            break
                            
                    except socket.timeout:
                        # Socket timeout is not an error, just continue
                        continue
                    except Exception as e:
                        print(cl_yellow(f"Error receiving data: {e}"))
                        if not self.running:
                            break
                            
                        # Don't immediately disconnect on errors
                        print(cl_yellow(f"Will attempt to continue..."))
                        time.sleep(1)
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
        try:
            self.connection.sendall((cmd + '\r\n').encode("UTF-8"))
        except:
            print(cl_red('Error: ') + "sending message thread failed")

    def recvmsg(self):
        header_len = 10
        msglength=0

        byt_len = ""
        byt_len = self.connection.recv(header_len)
        diff_header = header_len - len(byt_len)
        while (diff_header > 0):
            byt_len.extend(self.connection.recv(diff_header))
            diff_header= header_len-len(byt_len)

        msglength = int(byt_len.decode("utf-8")) + 1   #include crocodile and space
        msg = ""

        if(msglength>0 and msglength<5000):
            msg = self.connection.recv(msglength)
            diff_msg = msglength - len(msg)
            while(diff_msg>0):
                newmsg = self.connection.recv(diff_msg)
                msg.extend(newmsg)
                diff_msg = msglength - len(msg)
        return msg
