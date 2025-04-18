#!/usr/bin/env python3

import socket
import sys

def create_test_server(port, use_udp=False):
    if use_udp:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"Starting UDP test server on port {port}...")
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Starting TCP test server on port {port}...")
    
    # Bind to all interfaces
    server_address = ('0.0.0.0', port)
    sock.bind(server_address)
    
    if not use_udp:
        sock.listen(1)
        print(f"Listening on port {port}...")
        
        while True:
            print("Waiting for connection...")
            connection, client_address = sock.accept()
            print(f"Connection from {client_address}")
            
            try:
                while True:
                    data = connection.recv(1024)
                    if data:
                        print(f"Received: {data}")
                        connection.sendall(b"ACK")
                    else:
                        break
            finally:
                connection.close()
                print("Connection closed")
    else:
        print(f"Waiting for UDP data on port {port}...")
        while True:
            data, address = sock.recvfrom(1024)
            print(f"Received {len(data)} bytes from {address}")
            print(f"Data: {data}")
            sock.sendto(b"ACK", address)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: test_server.py <port> [udp]")
        sys.exit(1)
        
    port = int(sys.argv[1])
    use_udp = len(sys.argv) > 2 and sys.argv[2].lower() == "udp"
    
    try:
        create_test_server(port, use_udp)
    except KeyboardInterrupt:
        print("Server stopped")
