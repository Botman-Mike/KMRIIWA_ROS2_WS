#!/usr/bin/env python3

import socket
import sys
import time
import threading
import argparse

# Define ANSI colors for better output
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
CYAN = '\033[36m'
RESET = '\033[0m'

def hexdump(data, max_bytes=64):
    """Create a hexdump of binary data"""
    if not data:
        return "Empty data"
        
    hex_dump = ""
    if len(data) > max_bytes:
        hex_dump = " ".join([f"{b:02x}" for b in data[:max_bytes]]) + f"... ({len(data)} bytes total)"
    else:
        hex_dump = " ".join([f"{b:02x}" for b in data])
    
    # Try to decode as ASCII where possible
    ascii_text = ""
    for b in data[:max_bytes]:
        if 32 <= b <= 126:  # Printable ASCII
            ascii_text += chr(b)
        else:
            ascii_text += "."
            
    return f"HEX: {hex_dump}\nASCII: {ascii_text}"

def create_tcp_server(port):
    """Create a TCP server that logs all received data"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to all interfaces
    server_address = ('0.0.0.0', port)
    print(f"{CYAN}Starting TCP capture server on port {port}...{RESET}")
    sock.bind(server_address)
    sock.listen(1)
    
    print(f"{GREEN}Server listening on port {port}{RESET}")
    
    try:
        while True:
            print(f"Waiting for connection on port {port}...")
            connection, client_address = sock.accept()
            print(f"{GREEN}Client connected: {client_address}{RESET}")
            
            try:
                # Start a heartbeat sender thread
                def send_heartbeat():
                    while True:
                        try:
                            msg = "heartbeat"
                            length = str(len(msg)).zfill(10)
                            connection.sendall(f"{length}{msg}".encode("utf-8"))
                            print(f"{YELLOW}Sent heartbeat{RESET}")
                        except Exception as e:
                            print(f"{RED}Error sending heartbeat: {e}{RESET}")
                            break
                        time.sleep(5)
                
                heartbeat_thread = threading.Thread(target=send_heartbeat, daemon=True)
                heartbeat_thread.start()
                
                # Receive data
                bytes_received = 0
                while True:
                    data = connection.recv(4096)
                    if data:
                        bytes_received += len(data)
                        timestamp = time.strftime("%H:%M:%S")
                        print(f"{CYAN}[{timestamp}] Received {len(data)} bytes from {client_address} (total: {bytes_received}){RESET}")
                        print(f"{hexdump(data)}")
                    else:
                        print(f"{YELLOW}Client {client_address} disconnected{RESET}")
                        break
            finally:
                connection.close()
                
    except KeyboardInterrupt:
        print(f"{YELLOW}Server stopped by user{RESET}")
    finally:
        sock.close()

def create_udp_server(port):
    """Create a UDP server that logs all received data"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind to all interfaces
    server_address = ('0.0.0.0', port)
    print(f"{CYAN}Starting UDP capture server on port {port}...{RESET}")
    sock.bind(server_address)
    
    print(f"{GREEN}Server listening on UDP port {port}{RESET}")
    
    try:
        bytes_received = 0
        while True:
            data, client_address = sock.recvfrom(4096)
            if data:
                bytes_received += len(data)
                timestamp = time.strftime("%H:%M:%S")
                print(f"{CYAN}[{timestamp}] Received {len(data)} bytes from {client_address} (total: {bytes_received}){RESET}")
                print(f"{hexdump(data)}")
                
                # Send a fake response to keep the robot happy
                sock.sendto(b"ACK", client_address)
    except KeyboardInterrupt:
        print(f"{YELLOW}Server stopped by user{RESET}")
    finally:
        sock.close()

def main():
    parser = argparse.ArgumentParser(description='Capture and log data from robot communication ports')
    parser.add_argument('port', type=int, help='Port number to capture')
    parser.add_argument('--udp', action='store_true', help='Use UDP instead of TCP')
    args = parser.parse_args()
    
    if args.udp:
        create_udp_server(args.port)
    else:
        create_tcp_server(args.port)

if __name__ == "__main__":
    main()
