#!/usr/bin/env python3
"""
Joystick controller for Betaflight SITL simulator via UDP.
Reads from /dev/input/js0 and sends RC data to SITL UDP port 9004.

Axis mapping:
  AXIS0 = ROLL
  AXIS1 = PITCH
  AXIS2 = THROTTLE
  AXIS3 = YAW
  AXIS5 = AUX2
  AXIS6 = AUX3
  BUTTON0 = AUX1 (ARM)
"""

import socket
import struct
import sys
import time
import select
from typing import Optional

# Connection settings
HOST = '127.0.0.1'
RC_PORT = 9004  # SITL RC input port

# Joystick device
JS_DEVICE = '/dev/input/js0'

# Joystick event structure
# struct js_event {
#     __u32 time;     /* event timestamp in milliseconds */
#     __s16 value;    /* value */
#     __u8 type;      /* event type */
#     __u8 number;    /* axis/button number */
# };
JS_EVENT_SIZE = 8
JS_EVENT_FORMAT = 'IhBB'

# Event types
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02

# RC channel values (microseconds)
RC_MIN = 1000
RC_MAX = 2000
RC_MID = 1500

# Channel indices
ROLL = 0
PITCH = 1
THROTTLE = 2
YAW = 3
AUX1 = 4
AUX2 = 5
AUX3 = 6
AUX4 = 7


class JoystickController:
    def __init__(self, host=HOST, port=RC_PORT, device=JS_DEVICE):
        self.host = host
        self.port = port
        self.device = device
        self.sock = None
        self.js_fd = None
        
        # RC channels state
        self.channels = [RC_MID] * 16
        self.channels[THROTTLE] = RC_MIN
        
        # Axis mapping
        self.axis_map = {
            0: ROLL,
            1: PITCH,
            2: THROTTLE,
            3: YAW,
            5: AUX2,
            6: AUX3,
        }
        
        # Button mapping
        self.button_map = {
            0: AUX1,
        }
        
        self.connect()
    
    def connect(self):
        """Connect to SITL UDP server and open joystick device"""
        # Connect UDP socket
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP controller ready, sending to {self.host}:{self.port}")
        except Exception as e:
            print(f"Failed to create UDP socket: {e}")
            sys.exit(1)
        
        # Open joystick device
        try:
            self.js_fd = open(self.device, 'rb')
            print(f"Joystick opened: {self.device}")
        except Exception as e:
            print(f"Failed to open joystick: {e}")
            sys.exit(1)
    
    def read_joystick(self):
        """Read joystick events and update RC channels"""
        try:
            data = self.js_fd.read(JS_EVENT_SIZE)
            if len(data) != JS_EVENT_SIZE:
                return
            
            time_ms, value, event_type, number = struct.unpack(JS_EVENT_FORMAT, data)
            
            # Ignore init events
            if event_type & 0x80:
                return
            
            # Handle axis events
            if event_type == JS_EVENT_AXIS:
                if number in self.axis_map:
                    channel = self.axis_map[number]
                    # Map -32767..32767 to 1000..2000
                    rc_value = int((value + 32767) * (RC_MAX - RC_MIN) / 65534) + RC_MIN
                    self.channels[channel] = max(RC_MIN, min(RC_MAX, rc_value))
            
            # Handle button events
            elif event_type == JS_EVENT_BUTTON:
                if number in self.button_map:
                    channel = self.button_map[number]
                    # Map 0/1 to 1000/2000
                    self.channels[channel] = RC_MAX if value else RC_MIN
        
        except Exception as e:
            print(f"Joystick read error: {e}")
    
    def send_rc(self):
        """Send RC channels via UDP"""
        timestamp = time.time()
        # Pack: double timestamp + 16 uint16_t channels
        data = struct.pack('<d' + 'H' * 16, timestamp, *self.channels)
        
        try:
            self.sock.sendto(data, (self.host, self.port))
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def print_status(self):
        """Print current channel values"""
        print(f"\rTHR:{self.channels[THROTTLE]:4d} "
              f"YAW:{self.channels[YAW]:4d} "
              f"PIT:{self.channels[PITCH]:4d} "
              f"ROL:{self.channels[ROLL]:4d} "
              f"AUX1:{'ARM' if self.channels[AUX1] > 1500 else 'DISARM':7s} "
              f"AUX2:{self.channels[AUX2]:4d} "
              f"AUX3:{self.channels[AUX3]:4d}",
              end='', flush=False)
    
    def close(self):
        """Close connections"""
        if self.sock:
            self.sock.close()
        if self.js_fd:
            self.js_fd.close()


def main():
    controller = JoystickController()
    
    print("\nBetaflight SITL Joystick Controller")
    print("=" * 60)
    print("Controls:")
    print("  AXIS0 (ROLL)  - Roll")
    print("  AXIS1 (PITCH) - Pitch")
    print("  AXIS2 (THROTTLE) - Throttle")
    print("  AXIS3 (YAW)   - Yaw")
    print("  AXIS5         - AUX2")
    print("  AXIS6         - AUX3")
    print("  BUTTON0       - AUX1 (ARM/DISARM)")
    print("=" * 60)
    print("\nStarting ...")
    
    try:
        while True:
            # Check if joystick has data
            if select.select([controller.js_fd], [], [], 0.01)[0]:
                controller.read_joystick()
            
            controller.send_rc()
            controller.print_status()
            time.sleep(0.01)  # 50 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        controller.close()
        print("\nDisconnected")


if __name__ == '__main__':
    main()
