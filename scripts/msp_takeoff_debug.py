#!/usr/bin/env python3
"""
Debug reader for Betaflight SITL TAKEOFF mode
Reads debug values when DEBUG_TAKEOFF mode is active
"""

import socket
import struct
import time
import sys

# Connection settings
HOST = '127.0.0.1'
MSP_PORT = 5762

# MSP Protocol constants
MSP_HEADER = b'$M<'
MSP_DEBUG = 254
MSP_SET_DEBUG_MODE = 255

class MSPDebugReader:
    def __init__(self, host=HOST, port=MSP_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.connect()
    
    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.host, self.port))
            time.sleep(0.5)
            print(f"Connected to {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to connect: {e}")
            sys.exit(1)
    
    def calculate_checksum(self, data: bytes) -> int:
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum
    
    def send_msp_request(self, command: int, data=b'') -> bytes:
        size = len(data)
        checksum = size ^ command
        for byte in data:
            checksum ^= byte
        
        request = MSP_HEADER + struct.pack('BB', size, command) + data + struct.pack('B', checksum)
        
        try:
            self.sock.sendall(request)
            response = self.sock.recv(100)
            if not response or not response.startswith(b'$M>'):
                return None
            
            resp_size = response[3]
            resp_cmd = response[4]
            
            if resp_cmd != command:
                return None
            
            return response[5:5+resp_size]
            
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Communication error: {e}")
            return None
    
    def set_debug_mode(self, mode: int):
        """Set debug mode (0-255)"""
        data = struct.pack('B', mode)
        return self.send_msp_request(MSP_SET_DEBUG_MODE, data)
    
    def read_debug(self):
        """Read debug values (8 x int16_t)"""
        data = self.send_msp_request(MSP_DEBUG)
        if data and len(data) >= 16:
            values = struct.unpack('<8h', data[:16])
            return values
        return None
    
    def close(self):
        if self.sock:
            self.sock.close()


def main():
    reader = MSPDebugReader()
    
    print("\nBetaflight TAKEOFF Debug Reader")
    print("=" * 60)
    print("Debug mode: TAKEOFF")
    print("  debug[0]: current altitude (cm)")
    print("  debug[1]: target altitude (cm)")
    print("  debug[2]: state (0=IDLE, 1=ARMED, 2=CLIMBING, 3=HOLDING)")
    print("  debug[3]: pitch angle (degrees * 10)")
    print("  debug[4]: takeoff throttle config (1000-2000)")
    print("Press Ctrl+C to stop\n")
    
    # Set debug mode to TAKEOFF (you need to find the actual mode number)
    # For now, just read debug values
    
    try:
        while True:
            debug_values = reader.read_debug()
            
            if debug_values:
                state_names = ['IDLE', 'ARMED', 'CLIMBING', 'HOLDING']
                state_name = state_names[debug_values[2]] if debug_values[2] < len(state_names) else 'UNKNOWN'
                sys.stdout.write("\r" + " " * 100 + "\r")
                print(f"ALT:{debug_values[0]:5d}cm  "
                      f"TGT:{debug_values[1]:5d}cm  "
                      f"PITCH:{debug_values[3]/10:4.1f}°  "
                      f"THR_CFG:{debug_values[4]:4d}  "
                      f"ST:{state_name}", end='')
                sys.stdout.flush()
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        reader.close()
        print("Disconnected")


if __name__ == '__main__':
    main()
