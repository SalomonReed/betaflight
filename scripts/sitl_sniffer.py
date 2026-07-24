#!/usr/bin/env python3
"""
UDP sniffer for Betaflight SITL.
Monitors traffic between Gazebo and SITL.
"""

import socket
import struct
import sys

# Port definitions
PORT_PWM = 9002      # SITL → Gazebo (servo_packet: 4 floats)
PORT_STATE = 9003    # Gazebo → SITL (fdm_packet: 15 doubles)
PORT_RC = 9004       # External → SITL (rc_packet)

# fdm_packet structure
FDM_FORMAT = '<' + 'd' * 18
FDM_SIZE = struct.calcsize(FDM_FORMAT)  # 144 bytes
FDM_FIELDS = [
    'timestamp',
    'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z',
    'imu_lin_acc_x', 'imu_lin_acc_y', 'imu_lin_acc_z',
    'quat_w', 'quat_x', 'quat_y', 'quat_z',
    'vel_x', 'vel_y', 'vel_z',
    'pos_x', 'pos_y', 'pos_z',
    'pressure'
]

# servo_packet structure (4 floats)
SERVO_FORMAT = '<' + 'f' * 4
SERVO_SIZE = struct.calcsize(SERVO_FORMAT)

# rc_packet structure (double + 16 uint16)
RC_FORMAT = '<d' + 'H' * 16
RC_SIZE = struct.calcsize(RC_FORMAT)


def decode_fdm(data):
    """Decode fdm_packet"""
    if len(data) != FDM_SIZE:
        return None
    values = struct.unpack(FDM_FORMAT, data)
    return dict(zip(FDM_FIELDS, values))


def decode_servo(data):
    """Decode servo_packet"""
    if len(data) != SERVO_SIZE:
        return None
    values = struct.unpack(SERVO_FORMAT, data)
    return {'motor_0': values[0], 'motor_1': values[1], 
            'motor_2': values[2], 'motor_3': values[3]}


def decode_rc(data):
    """Decode rc_packet"""
    if len(data) != RC_SIZE:
        return None
    values = struct.unpack(RC_FORMAT, data)
    return {'timestamp': values[0], 'channels': list(values[1:])}


def main():
    ports_to_listen = [PORT_PWM, PORT_STATE, PORT_RC]
    
    print("Betaflight SITL UDP Sniffer")
    print("=" * 50)
    print(f"Listening on ports: {ports_to_listen}")
    print(f"  Port {PORT_PWM}: SITL → Gazebo (servo/motor data)")
    print(f"  Port {PORT_STATE}: Gazebo → SITL (FDM state)")
    print(f"  Port {PORT_RC}: RC input → SITL")
    print("=" * 50)
    print()
    
    # Create UDP sockets for each port
    sockets = {}
    for port in ports_to_listen:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
            sock.settimeout(0.1)
            sockets[port] = sock
            print(f"✓ Listening on port {port}")
        except Exception as e:
            print(f"✗ Failed to bind port {port}: {e}")
    
    if not sockets:
        print("\nNo ports available. Is SITL already running?")
        sys.exit(1)
    
    print("\nWaiting for data...\n")
    
    try:
        while True:
            for port, sock in sockets.items():
                try:
                    data, addr = sock.recvfrom(1024)
                    
                    if port == PORT_STATE:
                        fdm = decode_fdm(data)
                        if fdm:
                            print(f"[9003] FDM from {addr}")
                            print(f"  Time: {fdm['timestamp']:.3f}")
                            print(f"  Ang Vel: ({fdm['imu_ang_vel_x']:.3f}, {fdm['imu_ang_vel_y']:.3f}, {fdm['imu_ang_vel_z']:.3f}) rad/s")
                            print(f"  Lin Acc: ({fdm['imu_lin_acc_x']:.3f}, {fdm['imu_lin_acc_y']:.3f}, {fdm['imu_lin_acc_z']:.3f}) m/s²")
                            print(f"  Quat: ({fdm['quat_w']:.3f}, {fdm['quat_x']:.3f}, {fdm['quat_y']:.3f}, {fdm['quat_z']:.3f})")
                            print(f"  Velocity: ({fdm['vel_x']:.3f}, {fdm['vel_y']:.3f}, {fdm['vel_z']:.3f}) m/s")
                            print(f"  Position: ({fdm['pos_x']:.3f}, {fdm['pos_y']:.3f}, {fdm['pos_z']:.3f}) m")
                            print(f"  Pressure: {fdm['pressure']:.1f} Pa")
                            print()
                    
                    elif port == PORT_PWM:
                        servo = decode_servo(data)
                        if servo:
                            print(f"[9002] Servo to {addr}")
                            print(f"  Motors: [{servo['motor_0']:.3f}, {servo['motor_1']:.3f}, {servo['motor_2']:.3f}, {servo['motor_3']:.3f}]")
                            print()
                    
                    elif port == PORT_RC:
                        rc = decode_rc(data)
                        if rc:
                            print(f"[9004] RC from {addr}")
                            print(f"  Time: {rc['timestamp']:.3f}")
                            print(f"  Channels: {rc['channels'][:8]}")
                            print()
                    
                    else:
                        print(f"[{port}] Unknown data ({len(data)} bytes) from {addr}")
                        print(f"  Raw: {data.hex()}")
                        print()
                
                except socket.timeout:
                    pass
    
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        for sock in sockets.values():
            sock.close()


if __name__ == '__main__':
    main()
