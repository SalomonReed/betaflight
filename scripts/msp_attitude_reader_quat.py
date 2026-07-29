#!/usr/bin/env python3
"""
MSP Attitude Reader for Betaflight
Reads MSP_ATTITUDE (108) and MSP_ATTITUDE_QUATERNION (167) from flight controller
Supports TCP connections (for SITL and network-connected FC)
"""

import socket
import struct
import time
import sys
import math
from typing import Optional, Tuple

# Connection settings
HOST = '127.0.0.1'
MSP_PORT = 5762  # SITL MSP port (UART1)

# MSP Protocol constants
MSP_HEADER = b'$M<'
MSP_ATTITUDE = 108
MSP_ATTITUDE_QUATERNION = 167


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
    
    def read_attitude_quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        """Read MSP_ATTITUDE_QUATERNION (167) - returns quaternion (w, x, y, z)"""
        data = self.send_msp_request(MSP_ATTITUDE_QUATERNION)
        if data and len(data) >= 8:
            # Data format: 4 x int16_t (w, x, y, z) scaled by 0x7FFF
            w, x, y, z = struct.unpack('<hhhh', data[:8])
            q_scale = 0x7FFF
            return (w / q_scale, x / q_scale, y / q_scale, z / q_scale)
        return None
    
    def close(self):
        """Close TCP connection"""
        if self.sock:
            self.sock.close()


def quaternion_to_euler(q0, q1, q2, q3) -> Tuple[float, float, float]:
    """Convert quaternion to euler angles (roll, pitch, yaw) in degrees"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q0 * q3 + q1 * q2)
    cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    # Convert to degrees
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def main():
    reader = MSPReader(HOST, MSP_PORT)
    
    print("\nBetaflight MSP Attitude Reader")
    print("=" * 60)
    print(f"Connected to {HOST}:{MSP_PORT}")
    print("Reading MSP_ATTITUDE (108) and MSP_ATTITUDE_QUATERNION (167)")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            # Read Euler angles (roll, pitch, yaw)
            attitude = reader.read_attitude()
            
            # Read quaternion
            quaternion = reader.read_attitude_quaternion()
            
            # Display results
            sys.stdout.write("\r" + " " * 100 + "\r")  # Clear line
            
            if attitude:
                roll, pitch, yaw = attitude
                print(f"ATTITUDE: R:{roll:7.1f}° P:{pitch:7.1f}° Y:{yaw:7.1f}°", end='  ')
            
            if quaternion:
                q0, q1, q2, q3 = quaternion
                print(f"QUAT: [{q0:6.3f} {q1:6.3f} {q2:6.3f} {q3:6.3f}]", end='  ')
                
                # Convert quaternion to euler for comparison
                q_roll, q_pitch, q_yaw = quaternion_to_euler(q0, q1, q2, q3)
                print(f"→ R:{q_roll:7.1f}° P:{q_pitch:7.1f}° Y:{q_yaw:7.1f}°", end='')
            
            sys.stdout.flush()
            time.sleep(0.01)  # 10 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        reader.close()
        print("Disconnected")


if __name__ == '__main__':
    main()
