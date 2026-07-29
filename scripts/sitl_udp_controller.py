#!/usr/bin/env python3
"""
Keyboard controller for Betaflight SITL simulator via UDP.
Sends RC data directly to SITL UDP port 9004.
"""

import socket
import struct
import sys
import time
import select

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
AUX5 = 8


class UDPController:
    def __init__(self, host='127.0.0.1', port=9004):
        self.host = host
        self.port = port
        self.sock = None
        self.channels = [RC_MID] * 16  # 16 channels total
        self.channels[THROTTLE] = THROTTLE_MIN
        self.armed = False
        self.aux_states = [False] * 5  # Track AUX1-AUX5 states
        self.connect()
    
    def connect(self):
        """Connect to SITL UDP server"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP controller ready, sending to {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to create socket: {e}")
            sys.exit(1)
    
    def send_rc(self):
        """Send RC channels via UDP"""
        # SITL expects rc_packet structure:
        # double timestamp;
        # uint16_t channels[16];
        
        timestamp = time.time()
        # Pack: double timestamp + 16 uint16_t channels
        data = struct.pack('<d' + 'H' * 16, timestamp, *self.channels)
        
        try:
            self.sock.sendto(data, (self.host, self.port))
            return True
        except Exception as e:
            print(f"\nSend error: {e}")
            return False
    
    def arm(self):
        """Arm the craft (AUX1 high)"""
        self.channels[AUX1] = RC_MAX
        self.armed = True
        self.aux_states[0] = True
        print("ARMED")
    
    def disarm(self):
        """Disarm the craft (AUX1 low)"""
        self.channels[AUX1] = RC_MIN
        self.armed = False
        self.aux_states[0] = False
        print("DISARMED")
    
    def toggle_aux(self, aux_num):
        """Toggle AUX channel (1-5) between RC_MIN and RC_MAX"""
        if aux_num < 1 or aux_num > 5:
            return
        
        idx = aux_num - 1
        channel_idx = AUX1 + idx
        
        if self.aux_states[idx]:
            self.channels[channel_idx] = RC_MIN
            self.aux_states[idx] = False
        else:
            self.channels[channel_idx] = RC_MAX
            self.aux_states[idx] = True
        
        # Update armed state if AUX1 was toggled
        if aux_num == 1:
            self.armed = self.aux_states[0]
    
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
        aux_str = " ".join([f"{i+1}:{'H' if self.aux_states[i] else 'L'}" for i in range(5)])
        print(f"\rTHR:{self.channels[THROTTLE]:4d} "
              f"YAW:{self.channels[YAW]:4d} "
              f"PIT:{self.channels[PITCH]:4d} "
              f"ROL:{self.channels[ROLL]:4d} "
              f"AUX:[{aux_str}]", 
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
    controller = UDPController()
    
    print("\nBetaflight SITL UDP Keyboard Controller")
    print("=" * 40)
    print("Controls:")
    print("  W/S     - Throttle up/down")
    print("  A/D     - Yaw left/right")
    print("  ↑/↓     - Pitch forward/back")
    print("  ←/→     - Roll left/right")
    print("  Space   - Arm/Disarm toggle (AUX1)")
    print("  1-5     - Toggle AUX1-AUX5")
    print("  C       - Center sticks")
    print("  Q       - Quit")
    print("=" * 40)
    print("\nStarting ...")
    
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
                    controller.adjust_pitch(50)
                elif key == '\x1b[B':  # Down arrow
                    controller.adjust_pitch(-50)
                elif key == '\x1b[D':  # Left arrow
                    controller.adjust_roll(-50)
                elif key == '\x1b[C':  # Right arrow
                    controller.adjust_roll(50)
                elif key == 'c' or key == 'C':
                    controller.center_sticks()
                elif key in '12345':
                    controller.toggle_aux(int(key))
            
            controller.send_rc()
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
