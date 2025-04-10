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
        self.heartbeat_timeout = 10  # seconds
        self.running = True

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
        initialization_grace_period = 5  # Give the socket this many seconds to connect before reporting status
        start_time = time.time()
        
        while self.running:
            # During initialization, don't report down status
            if last_reported_status is None and time.time() - start_time < initialization_grace_period:
                time.sleep(1)
                continue
                
            # First time status check after grace period
            if last_reported_status is None:
                last_reported_status = self.isconnected
                # Only report if the connection is up after grace period
                if self.isconnected:
                    print(cl_green(f"Connection established for {self.node_name}"))
                # Don't report initial "down" - it's expected during startup
                    
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
                self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.tcp.bind(server_address)
                self.tcp.listen(3)
                self.tcp.settimeout(30)  # 30 second timeout for accept()

                with self.connection_lock:
                    self.connection, client_address = self.tcp.accept()
                    self.tcp.settimeout(0.5)  # Increased timeout for better stability
                    self.isconnected = True
                    self.last_heartbeat = time.time()

                print(cl_green(f'Connected to client at {client_address}'))
                attempt = 0  # Reset attempt counter on successful connection
                time.sleep(1)

                # Main data processing loop
                while self.isconnected and self.running:
                    try:
                        self.last_heartbeat = time.time()
                        data = self.recvmsg()
                        
                        # Log the size of incoming data for debugging
                        if data:
                            print(f"Received data of size {len(data)} bytes from {self.node_name}")
                        else:
                            raise socket.error("Connection closed by remote host")
                        
                        # Check if this is a heartbeat-only message
                        try:
                            data_str = data.decode("utf-8")
                            if data_str.strip() in ["heartbeat", "ping", ""]:
                                continue  # Skip processing for heartbeat messages
                        except UnicodeDecodeError:
                            # If it can't be decoded as text, just use it as a heartbeat
                            continue

                        for pack in (data.decode("utf-8")).split(">"):
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
                    except socket.timeout:
                        # Socket timeout is not an error, just continue
                        continue
                    except Exception as e:
                        print(cl_yellow(f"Error processing data: {e}"))
                        with self.connection_lock:
                            self.isconnected = False
                        break

                print(cl_yellow(f"Connection lost, cleaning up..."))
                with self.connection_lock:
                    try:
                        if self.connection:
                            self.connection.close()
                        if self.tcp:
                            self.tcp.close()
                    except Exception as ex:
                        print(cl_red(f"Error during cleanup: {ex}"))

                if not self.running:
                    break

                print(cl_yellow(f"Attempting to reconnect in {self.reconnection_delay} seconds..."))
                time.sleep(self.reconnection_delay)
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
                    if self.tcp:
                        self.tcp.close()
                except:
                    pass
                
                continue

    def send(self, cmd):
        """Thread-safe send command"""
        if not self.isconnected:
            print(cl_yellow(f"Cannot send command, not connected"))
            return False

        try:
            with self.connection_lock:
                if self.isconnected and self.connection:
                    self.connection.sendall((cmd + '\r\n').encode("UTF-8"))
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
