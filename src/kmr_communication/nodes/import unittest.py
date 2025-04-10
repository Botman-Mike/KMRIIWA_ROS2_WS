import unittest
import socket
import threading
import time
from unittest.mock import patch, MagicMock
from ..nodes.udpSocket import UDPSocket

#!/usr/bin/env python3


# Import the class to test using relative import

class TestUDPSocket(unittest.TestCase):
    
    @patch('socket.socket')
    def test_connect_to_socket_successful_connection(self, mock_socket):
        # Setup mock socket
        mock_udp = MagicMock()
        mock_socket.return_value = mock_udp
        
        # Mock recvfrom to return data to trigger a connection
        mock_udp.recvfrom.return_value = (b"test data", ("192.168.1.100", 30000))
        
        # Create UDPSocket instance
        udp_socket = UDPSocket("127.0.0.1", 30002, "test_node")
        
        # Give some time for the connection thread to run
        time.sleep(0.5)
        
        # Check that the socket was properly configured
        mock_socket.assert_called_once_with(socket.AF_INET, socket.SOCK_DGRAM)
        mock_udp.settimeout.assert_called_once_with(1.0)
        mock_udp.setsockopt.assert_called_once()
        mock_udp.bind.assert_called_once_with(("127.0.0.1", 30002))
        
        # Check that connection was established
        self.assertTrue(udp_socket.isconnected)
        self.assertEqual(udp_socket.client_address, ("192.168.1.100", 30000))
        
        # Check that the hello message was sent
        mock_udp.sendto.assert_called_with(b"hello KUKA", ("192.168.1.100", 30000))
        
        # Clean up
        udp_socket.close()

    @patch('socket.socket')
    def test_connect_to_socket_timeout(self, mock_socket):
        # Setup mock socket
        mock_udp = MagicMock()
        mock_socket.return_value = mock_udp
        
        # Mock recvfrom to raise timeout
        mock_udp.recvfrom.side_effect = socket.timeout
        
        # Create UDPSocket instance with short timeout
        udp_socket = UDPSocket("127.0.0.1", 30002, "test_node")
        
        # Replace the actual reconnection delay with a very short one for testing
        udp_socket.reconnection_delay = 0.1
        
        # Give time for the connection attempt and timeout
        time.sleep(0.5)
        
        # Check that connection was not established
        self.assertFalse(udp_socket.isconnected)
        
        # Clean up
        udp_socket.close()

    @patch('socket.socket')
    def test_connect_to_socket_exception_handling(self, mock_socket):
        # Setup mock socket
        mock_udp = MagicMock()
        mock_socket.return_value = mock_udp
        
        # Mock bind to raise an exception
        mock_udp.bind.side_effect = socket.error("Test error")
        
        # Create UDPSocket instance
        udp_socket = UDPSocket("127.0.0.1", 30002, "test_node")
        udp_socket.reconnection_delay = 0.1
        udp_socket.max_reconnection_attempts = 2
        
        # Give time for connection attempts
        time.sleep(0.5)
        
        # Check that connection was not established
        self.assertFalse(udp_socket.isconnected)
        
        # Verify that bind was called
        mock_udp.bind.assert_called_with(("127.0.0.1", 30002))
        
        # Clean up
        udp_socket.close()

    @patch('socket.socket')
    def test_heartbeat_timeout(self, mock_socket):
        # Setup mock socket
        mock_udp = MagicMock()
        mock_socket.return_value = mock_udp
        
        # Mock recvfrom to return data initially to establish connection
        mock_udp.recvfrom.return_value = (b"test data", ("192.168.1.100", 30000))
        
        # Create UDPSocket instance with very short heartbeat timeout
        udp_socket = UDPSocket("127.0.0.1", 30002, "test_node")
        udp_socket.heartbeat_timeout = 0.2  # Short timeout for testing
        
        # Give time for connection to establish
        time.sleep(0.1)
        
        # Verify connection was established
        self.assertTrue(udp_socket.isconnected)
        
        # Now let the heartbeat timeout
        time.sleep(0.3)
        
        # Check that connection was marked as down
        self.assertFalse(udp_socket.isconnected)
        
        # Clean up
        udp_socket.close()
    
    @patch('socket.socket')
    def test_data_processing(self, mock_socket):
        # Setup mock socket
        mock_udp = MagicMock()
        mock_socket.return_value = mock_udp
        
        # Prepare a sequence of data to be returned by recvfrom
        mock_udp.recvfrom.side_effect = [
            (b"test data", ("192.168.1.100", 30000)),  # Initial connection
            (b"odometry 1.0 2.0 3.0", ("192.168.1.100", 30000)),  # Odometry data
            (b"laserScan data 1801 values", ("192.168.1.100", 30000)),  # LaserScan B1
            (b"kmp_statusdata status1 status2", ("192.168.1.100", 30000)),  # KMP status
            socket.timeout  # End the loop with a timeout
        ]
        
        # Create UDPSocket instance
        udp_socket = UDPSocket("127.0.0.1", 30002, "test_node")
        
        # Give time for connection and processing data
        time.sleep(0.5)
        
        # Check that connection was established
        self.assertTrue(udp_socket.isconnected)
        
        # Verify the data was processed correctly
        self.assertEqual(udp_socket.odometry, ["odometry", "1.0", "2.0", "3.0"])
        self.assertEqual(len(udp_socket.laserScanB1), 1)
        self.assertEqual(udp_socket.kmp_statusdata, ["kmp_statusdata", "status1", "status2"])
        
        # Clean up
        udp_socket.close()

if __name__ == '__main__':
    unittest.main()