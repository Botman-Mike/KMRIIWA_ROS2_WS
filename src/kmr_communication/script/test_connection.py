#!/usr/bin/env python3
# Script to test TCP connection to the ROS nodes

import socket
import sys
import time
import threading

def send_heartbeats(sock, interval=5):
    """Send heartbeats to keep connection alive"""
    while True:
        try:
            # Format according to protocol: 10-digit length prefix + "heartbeat"
            msg = "heartbeat"
            length = str(len(msg)).zfill(10)  # 10-digit length prefix
            heartbeat_msg = length + msg
            sock.sendall(heartbeat_msg.encode("UTF-8"))
            print(f"Sent heartbeat")
        except Exception as e:
            print(f"Error sending heartbeat: {e}")
            break
        time.sleep(interval)

def test_tcp_connection(host, port, duration):
    """Test TCP connection to given host:port for specified duration"""
    try:
        # Create socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Set timeout for connection
        sock.settimeout(10)
        
        print(f"Connecting to {host}:{port}...")
        sock.connect((host, port))
        print(f"Connected to {host}:{port}")
        
        # Start heartbeat thread
        heartbeat_thread = threading.Thread(target=send_heartbeats, args=(sock,), daemon=True)
        heartbeat_thread.start()
        
        # Keep connection open for specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                # Try to receive data
                data = sock.recv(4096)
                if data:
                    try:
                        print(f"Received: {data.decode('utf-8')}")
                    except UnicodeDecodeError:
                        print(f"Received binary data of {len(data)} bytes")
            except socket.timeout:
                # Timeout is expected
                pass
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
                
            # Print status every 5 seconds
            if int(time.time() - start_time) % 5 == 0:
                print(f"Connection active for {int(time.time() - start_time)} seconds...")
                time.sleep(1)  # Avoid repeated messages
                
        print(f"Test completed after {duration} seconds")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: test_connection.py <host> <port> [duration]")
        print("Example: test_connection.py 172.31.1.206 30002 60")
        sys.exit(1)
        
    host = sys.argv[1]
    port = int(sys.argv[2])
    duration = int(sys.argv[3]) if len(sys.argv) > 3 else 60  # Default 60 seconds
    
    test_tcp_connection(host, port, duration)
