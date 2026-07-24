#!/usr/bin/env python3
"""
Keyboard controller for Betaflight SITL simulator via MSP protocol.
Controls:
  W/S     - Throttle up/down
  A/D     - Yaw left/right
  Arrows  - Pitch/Roll
  Space   - Arm/Disarm toggle
  Q       - Quit
"""

import socket
import struct
import sys
import time
import select

# MSP Protocol constants
MSP_API_VERSION = 1
MSP_IDENT = 100
MSP_STATUS = 101
MSP_SET_RAW_RC = 200

# RC channel values (microseconds)
RC_MIN = 1000
RC_MAX = 2000
RC_MID = 1500
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000

# Channel indices
ROLL = 0
PITCH = 1
YAW = 3
THROTTLE = 2
AUX1 = 4  # Arm switch
AUX2 = 5
AUX3 = 6
AUX4 = 7


class MSPController:
    def __init__(self, host='127.0.0.1', port=5762):
        self.host = host
        self.port = port
        self.sock = None
        self.channels = [RC_MID] * 8
        self.channels[THROTTLE] = THROTTLE_MIN
        self.armed = False
        self.connect()
    
    def connect(self):
        """Connect to SITL MSP server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)  # 5 second timeout
            self.sock.connect((self.host, self.port))
            print(f"Connected to SITL at {self.host}:{self.port}")
            
            # Try to get MSP API version to verify connection
            time.sleep(0.5)
            if self.send_msp_query(MSP_API_VERSION):
                print("MSP connection verified")
            else:
                print("Warning: Could not verify MSP connection")
        except Exception as e:
            print(f"Failed to connect: {e}")
            sys.exit(1)
    
    def send_msp_query(self, command):
        """Send MSP query and try to read response"""
        self.send_msp(command)
        time.sleep(0.1)
        try:
            self.sock.settimeout(1.0)
            response = self.sock.recv(1024)
            self.sock.settimeout(5.0)
            if response and response.startswith(b'$M>'):
                return True
        except socket.timeout:
            print("MSP query timeout")
        except Exception as e:
            print(f"MSP query error: {e}")
        return False
    
    def send_msp(self, command, data=b''):
        """Send MSP v1 message"""
        # MSP v1 format: $M< + size + command + data + checksum
        size = len(data)
        checksum = size ^ command
        for byte in data:
            checksum ^= byte
        
        header = b'$M<'
        packet = header + struct.pack('BB', size, command) + data + struct.pack('B', checksum)
        
        try:
            self.sock.sendall(packet)
            return True
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"\nConnection lost: {e}")
            return False
        except Exception as e:
            print(f"\nUnexpected send error: {e}")
            return False
    
    def reconnect(self):
        """Reconnect to SITL"""
        print("Attempting to reconnect...")
        try:
            if self.sock:
                self.sock.close()
            time.sleep(1)
            self.connect()
            return True
        except Exception as e:
            print(f"Reconnect failed: {e}")
            return False
    
    def send_rc(self):
        """Send RC channels via MSP_SET_RAW_RC"""
        # Pack 8 channels as uint16_t (little-endian)
        data = struct.pack('<' + 'H' * 8, *self.channels)
        return self.send_msp(MSP_SET_RAW_RC, data)
    
    def arm(self):
        """Arm the craft (AUX1 high)"""
        self.channels[AUX1] = RC_MAX
        self.armed = True
        print("ARMED")
    
    def disarm(self):
        """Disarm the craft (AUX1 low)"""
        self.channels[AUX1] = RC_MIN
        self.armed = False
        print("DISARMED")
    
    def adjust_throttle(self, delta):
        """Adjust throttle value"""
        self.channels[THROTTLE] = max(THROTTLE_MIN, 
                                      min(THROTTLE_MAX, self.channels[THROTTLE] + delta))
    
    def adjust_yaw(self, delta):
        """Adjust yaw value"""
        self.channels[YAW] = max(RC_MIN, min(RC_MAX, self.channels[YAW] + delta))
    
    def adjust_pitch(self, delta):
        """Adjust pitch value"""
        self.channels[PITCH] = max(RC_MIN, min(RC_MAX, self.channels[PITCH] + delta))
    
    def adjust_roll(self, delta):
        """Adjust roll value"""
        self.channels[ROLL] = max(RC_MIN, min(RC_MAX, self.channels[ROLL] + delta))
    
    def center_sticks(self):
        """Center all sticks except throttle"""
        self.channels[ROLL] = RC_MID
        self.channels[PITCH] = RC_MID
        self.channels[YAW] = RC_MID
    
    def print_status(self):
        """Print current channel values"""
        print(f"\rTHR:{self.channels[THROTTLE]:4d} "
              f"YAW:{self.channels[YAW]:4d} "
              f"PIT:{self.channels[PITCH]:4d} "
              f"ROL:{self.channels[ROLL]:4d} "
              f"AUX1:{'ARM' if self.armed else 'DISARM':7s}", 
              end='', flush=True)
    
    def close(self):
        """Close connection"""
        if self.sock:
            self.sock.close()


def get_key():
    """Get single keypress (Unix/Linux)"""
    import tty
    import termios
    
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0.1)[0]:
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # Escape sequence
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    ch3 = sys.stdin.read(1)
                    return f'\x1b[{ch3}'
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    controller = MSPController()
    
    print("\nBetaflight SITL Keyboard Controller")
    print("=" * 40)
    print("Controls:")
    print("  W/S     - Throttle up/down")
    print("  A/D     - Yaw left/right")
    print("  ↑/↓     - Pitch forward/back")
    print("  ←/→     - Roll left/right")
    print("  Space   - Arm/Disarm toggle")
    print("  C       - Center sticks")
    print("  Q       - Quit")
    print("=" * 40)
    print("\nStarting in 3 seconds...")
    time.sleep(3)
    
    try:
        while True:
            key = get_key()
            
            if key:
                if key == 'q' or key == 'Q':
                    break
                elif key == ' ':  # Space - toggle arm
                    if controller.armed:
                        controller.disarm()
                    else:
                        controller.arm()
                elif key == 'w' or key == 'W':
                    controller.adjust_throttle(50)
                elif key == 's' or key == 'S':
                    controller.adjust_throttle(-50)
                elif key == 'a' or key == 'A':
                    controller.adjust_yaw(-50)
                elif key == 'd' or key == 'D':
                    controller.adjust_yaw(50)
                elif key == '\x1b[A':  # Up arrow
                    controller.adjust_pitch(-50)
                elif key == '\x1b[B':  # Down arrow
                    controller.adjust_pitch(50)
                elif key == '\x1b[D':  # Left arrow
                    controller.adjust_roll(-50)
                elif key == '\x1b[C':  # Right arrow
                    controller.adjust_roll(50)
                elif key == 'c' or key == 'C':
                    controller.center_sticks()
            
            if not controller.send_rc():
                if not controller.reconnect():
                    print("Failed to reconnect, exiting...")
                    break
            else:
                controller.print_status()
            
            time.sleep(0.02)  # 50 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        controller.disarm()
        controller.close()
        print("\nDisconnected")


if __name__ == '__main__':
    main()
