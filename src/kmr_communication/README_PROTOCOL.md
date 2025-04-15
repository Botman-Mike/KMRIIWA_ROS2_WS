# KUKA-ROS2 Communication Protocol Implementation

This document describes how the communication protocol is implemented in this ROS2 package to interface with the KUKA KMR iiwa robots.

## Connection Types

The system uses two types of socket connections:

- **TCP**: Used for critical data requiring reliable delivery
  - Robot commands
  - Status updates
  - Heartbeat messages
  
- **UDP**: Used for high-frequency sensor data
  - LBR joint states/sensors
  - KMP laser scanner
  - KMP odometry

## Port Assignments

| Port   | Data Type         | Connection Type |
|--------|-------------------|----------------|
| 30001  | KMP Status        | TCP            |
| 30002  | KMP Command       | TCP            |
| 30003  | KMP Laser         | UDP            |
| 30004  | KMP Odometry      | UDP            |
| 30005  | LBR Command       | TCP            |
| 30006  | LBR Status        | TCP            |
| 30007  | LBR Sensors       | UDP            |

## Heartbeat Protocol

- **TCP connections**: 10-digit length prefix + "heartbeat"
- **UDP connections**: Simple "heartbeat" message
- **Frequency**: Every 5 seconds
- **Timeout**: 10 seconds

## Performance Optimizations

- Separate UDP sockets for different sensor types
- Increased UDP buffer sizes (65535 bytes)
- Non-blocking socket operations
- Thread priority management based on robot state

## Error Recovery

- Automatic reconnection attempts
- Graceful degradation during packet loss
- Independent socket monitoring
- Safety state integration

## Implementation Details

The connection handling is implemented in two main classes:

1. `TCPSocket` in `tcpSocket.py` - Handles reliable TCP connections
2. `UDPSocket` in `udpSocket.py` - Handles high-throughput UDP connections

Each ROS2 node creates an appropriate socket instance based on the data type it handles.
