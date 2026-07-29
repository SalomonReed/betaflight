#!/usr/bin/env python3
"""
MSP Attitude Reader for Betaflight
Reads MSP_ATTITUDE (108) and MSP_RAW_IMU (102) from flight controller
Supports TCP connections (for SITL and network-connected FC)
"""

import socket
import struct
import time
import sys
from typing import Optional, Tuple

# Connection settings
HOST = '127.0.0.1'
MSP_PORT = 5762  # SITL MSP port (UART2)

# MSP Protocol constants
MSP_HEADER = b'$M<'
MSP_ATTITUDE = 108
MSP_RAW_IMU = 102


class MSPReader:
    def __init__(self, host: str = HOST, port: int = MSP_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.connect()
    
    def connect(self):
        """Connect to flight controller via TCP"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            time.sleep(0.5)  # Wait for connection to stabilize
            print(f"Connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to connect: {e}")
            sys.exit(1)
    
    def calculate_checksum(self, data: bytes) -> int:
        """Calculate MSP checksum (XOR of all bytes)"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def send_msp_request(self, command: int) -> Optional[bytes]:
        """Send MSP request and receive response via TCP"""
        # Build request: $M< + size(0) + command + checksum
        request = MSP_HEADER + struct.pack('BB', 0, command)
        checksum = self.calculate_checksum(struct.pack('B', command))
        request += struct.pack('B', checksum)
        
        try:
            # Send request
            self.sock.sendall(request)
            
            # Read response
            response = self.sock.recv(100)
            if not response:
                return None
            
            # Parse response: $M> + size + command + data + checksum
            if not response.startswith(b'$M>'):
                return None
            
            size = response[3]
            cmd = response[4]
            
            if cmd != command:
                return None
            
            data = response[5:5+size]
            return data
            
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Communication error: {e}")
            return None
    
    def read_attitude(self) -> Optional[Tuple[float, float, float]]:
        """Read MSP_ATTITUDE (108) - returns roll, pitch, yaw in degrees"""
        data = self.send_msp_request(MSP_ATTITUDE)
        if data and len(data) >= 6:
            # Data format: int16_t roll, pitch, yaw (in 0.1 degrees)
            roll, pitch, yaw = struct.unpack('<hhh', data[:6])
            return (roll / 10.0, pitch / 10.0, yaw / 10.0)
        return None
    
    def read_raw_imu(self) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int], Tuple[int, int, int]]]:
        """Read MSP_RAW_IMU (102) - returns (acc, gyro, mag) tuples"""
        data = self.send_msp_request(MSP_RAW_IMU)
        if data and len(data) >= 18:
            # Data format: 9 x int16_t (acc x/y/z, gyro x/y/z, mag x/y/z)
            values = struct.unpack('<9h', data[:18])
            acc = (values[0], values[1], values[2])
            gyro = (values[3], values[4], values[5])
            mag = (values[6], values[7], values[8])
            return (acc, gyro, mag)
        return None
    
    def close(self):
        """Close TCP connection"""
        if self.sock:
            self.sock.close()


def main():
    reader = MSPReader(HOST, MSP_PORT)
    
    print("\nBetaflight MSP Attitude Reader")
    print("=" * 60)
    print(f"Connected to {HOST}:{MSP_PORT}")
    print("Reading MSP_ATTITUDE (108) and MSP_RAW_IMU (102)")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            # Read Euler angles (roll, pitch, yaw)
            attitude = reader.read_attitude()
            
            # Read raw IMU data
            imu = reader.read_raw_imu()
            
            # Display results
            sys.stdout.write("\r" + " " * 100 + "\r")  # Clear line
            
            if attitude:
                roll, pitch, yaw = attitude
                print(f"ATTITUDE: R:{roll:7.1f}° P:{pitch:7.1f}° Y:{yaw:7.1f}°", end='  ')
            
            if imu:
                acc, gyro, mag = imu
                print(f"GYRO: ({gyro[0]:5d} {gyro[1]:5d} {gyro[2]:5d})", end='')
            
            sys.stdout.flush()
            time.sleep(0.1)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        reader.close()
        print("Disconnected")


if __name__ == '__main__':
    main()
