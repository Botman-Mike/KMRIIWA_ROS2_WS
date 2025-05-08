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


class TCPSocket:
    def __init__(self, ip, port,node):
        # ROS2 logger for this socket
        self.logger = get_logger(node)
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

        # Setup listening socket once at startup
        self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_socket.bind((self.ip, self.port))
        self.listen_socket.listen(10)
        # log that we’re listening with TCP_NODELAY disabled until accept
        self.logger.info(f"{self.node_name}|port {self.port}|TCP_NODELAY=1|listening")
        self.listen_socket.settimeout(600)

        # Start server thread as daemon
        threading.Thread(target=self.connect_to_socket, daemon=True).start()

        #Data
        self.odometry = []
        self.laserScanB1 = []
        self.laserScanB4 = []
        self.kmp_statusdata = None
        self.lbr_statusdata = None
        self.lbr_sensordata = []

    def close(self):
        self.logger.info(f"[DEBUG] close() called for {self.node_name}, isconnected={self.isconnected}, running={self.running}")
        # Stop server loop and close sockets
        self.running = False
        self.isconnected = False
        if self.connection:
            try:
                self.connection.close()
                self.logger.info(f"[DEBUG] Socket closed for {self.node_name}")
            except Exception as e:
                self.logger.error(f"[DEBUG] Exception during socket close: {e}")
        # Close listening socket if open
        if hasattr(self, 'listen_socket'):
            try:
                self.listen_socket.close()
            except Exception:
                pass

    def connect_to_socket(self):
        """Accept and handle incoming connections in a loop"""
        attempt = 0

        while self.running:
            # Wait for incoming client
            self.logger.info(f"{self.node_name}|port {self.port} waiting for connections on {self.ip}:{self.port}")
            with self.connection_lock:
                self.connection, client_address = self.listen_socket.accept()
            # disable Nagle’s algorithm
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.logger.info(f"{self.node_name}|port {self.port}|TCP_NODELAY=1|accepted connection from {client_address}")

            # Increase socket buffer sizes for performance
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1048576)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1048576)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            if hasattr(socket, 'TCP_KEEPIDLE'):
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
            if hasattr(socket, 'TCP_KEEPINTVL'):
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
            if hasattr(socket, 'TCP_KEEPCNT'):
                self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 6)
            self.connection.settimeout(600)  # 10 minute operations timeout
            self.isconnected = True
            self.last_heartbeat = time.time()

            self.logger.info(f'Connected to client at {client_address}')
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
                                    heartbeat_msg = length + " " + msg
                                    self.connection.sendall(heartbeat_msg.encode("UTF-8"))
                                    last_heartbeat_send = current_time
                    except Exception as e:
                        self.logger.warn(f"Heartbeat send error: {e}")
                        try:
                            self.connection.close()
                        except Exception:
                            pass
                        self.isconnected = False
                        return
            
            # Start heartbeat thread
            threading.Thread(target=send_heartbeat, daemon=True).start()

            # Main data processing loop
            while self.isconnected and self.running:
                try:
                    msg = self.recvmsg()
                    if msg:
                        self.last_heartbeat = time.time()
                        # Log the raw message received for debugging
                        try:
                            data_str = msg.decode('utf-8', errors='replace').strip()
                            self.logger.info(f"[RECEIVED MESSAGE] {self.node_name}: '{data_str}'")
                            # Try to decode as UTF-8, but handle binary data gracefully
                            try:
                                data_str = msg.decode('utf-8').strip()
                                
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
                        except UnicodeDecodeError:
                            self.logger.error(f"[RECEIVED MESSAGE] {self.node_name}: <binary data>")
                            pass
                    else:
                        self.logger.warn(f"Client disconnected - empty data received")
                        self.logger.warn(f"[DEBUG] In main loop: isconnected={self.isconnected}, running={self.running}, connection={self.connection}")
                        # Close this client and continue to accept new ones
                        with self.connection_lock:
                            if self.connection:
                                self.connection.close()
                            self.isconnected = False
                        self.logger.error(f"[DISCONNECT] Reason: Received empty data (peer closed connection) in {self.node_name}")
                        continue
                        
                except socket.timeout:
                    continue
                # Handle socket errors specifically before other exceptions
                except socket.error as e:
                    err_type = type(e).__name__
                    self.logger.warn(f'Connection error: {err_type}: {e}')
                    attempt += 1
                    if e.errno == 111:
                        self.logger.warn(f"  → The robot may not be listening on this port yet")
                    elif e.errno == 110:
                        self.logger.warn(f"  → Network route exists but robot not responding")
                    self.logger.error(f"[DISCONNECT] Reason: socket.error during connect: {e} in {self.node_name}")
                    if attempt >= self.max_reconnection_attempts:
                        self.logger.error(f'Maximum reconnection attempts reached. Waiting longer...')
                        time.sleep(self.reconnection_delay * 5)
                        attempt = 0
                    else:
                        time.sleep(self.reconnection_delay)
                    try:
                        if self.connection:
                            self.connection.close()
                    except Exception:
                        pass
                    self.isconnected = False
                    continue
                except Exception as e:
                    self.logger.warn(f"Error receiving data: {e}")
                    self.logger.warn(f"[DEBUG] Exception in main loop: isconnected={self.isconnected}, running={self.running}, connection={self.connection}")
                    # NEW: Log reason for disconnect
                    self.logger.error(f"[DISCONNECT] Reason: Exception in main loop: {e} in {self.node_name}")
                    # Merged accept-error logging
                    self.logger.error(f"Error while accepting connection: {e}")
                    if not self.running:
                        break
                        
                    # Don't immediately disconnect on errors
                    self.logger.warn(f"Will attempt to continue...")
                    try:
                        self.connection.close()
                    except:
                        pass
                    self.isconnected = False
                    time.sleep(1)
                    continue
                        
            # ...remaining code for accept loop, no outer generic exception handler

    def send(self, cmd):
        try:
            msg = cmd + '\r\n'
            length = str(len(msg)).zfill(10)
            msg_with_prefix = length + " " + msg
            self.connection.sendall(msg_with_prefix.encode("UTF-8"))
        except:
            self.logger.error('Error: sending message thread failed')

    def recvmsg(self):
        # synchronize on 10-digit length header + space (sliding window)
        buf = bytearray()
        # track resync thresholds
        scanned = 0
        start_time = time.time()
        max_resync_bytes = 2048  # bytes threshold for header re-sync
        max_resync_time = 5.0  # seconds
        # read until we have at least 11 bytes
        while len(buf) < 11:
            chunk = self.connection.recv(11 - len(buf))
            if not chunk:
                # connection closed or no data
                self.isconnected = False
                return b""
            buf.extend(chunk)
        # slide window until header matches: 10 digits + space
        while not (all(48 <= b <= 57 for b in buf[0:10]) and buf[10] == 0x20):
            # check thresholds for reset
            if scanned > max_resync_bytes or time.time() - start_time > max_resync_time:
                self.logger.warn(f"{self.node_name}|port {self.port} header sync exceeded ({scanned} bytes, {time.time()-start_time:.1f}s), resetting connection")
                try:
                    self.connection.close()
                except:
                    pass
                self.isconnected = False
                return b""
            # drop one byte and read one more
            buf.pop(0)
            new = self.connection.recv(1)
            if not new:
                self.isconnected = False
                return b""
            buf.extend(new)
            scanned += 1
        # extract header length
        header = bytes(buf[0:10])
        try:
            length = int(header.decode('utf-8'))
        except ValueError:
            # invalid header, abort
            self.logger.error(f"{self.node_name}|port {self.port} invalid header '{header}'")
            self.connection.close()
            self.isconnected = False
            return b""
        self.logger.debug(f"{self.node_name}|port {self.port} received header: '{header.decode()}' length={length}")
        # read exact data length
        data = bytearray(length)
        view = memoryview(data)
        idx = 0
        while idx < length:
            chunk = self.connection.recv(length - idx)
            if not chunk:
                self.logger.error(f"{self.node_name}|port {self.port} connection closed during data read")
                try:
                    self.connection.close()
                except Exception:
                    pass
                self.isconnected = False
                return b""
            view[idx:idx+len(chunk)] = chunk
            idx += len(chunk)
        # verify frame ends with CRLF to confirm correct sync
        data_bytes = bytes(data)
        if not data_bytes.endswith(b"\r\n"):
            self.logger.warn(f"{self.node_name}|port {self.port} frame tail invalid, resyncing")
            return self.recvmsg()
        self.logger.debug(f"{self.node_name}|port {self.port} read {length} bytes of data")
        return data_bytes
