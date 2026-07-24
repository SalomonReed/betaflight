#!/usr/bin/env python3
"""
Send fake FDM data to Betaflight SITL for testing barometer.
Sends fdm_packet structure to UDP port 9003.
"""

import socket
import struct
import time
import math

# fdm_packet structure (from target.h):
# double timestamp;
# double imu_angular_velocity_rpy[3];  // rad/s
# double imu_linear_acceleration_xyz[3]; // m/s/s NED
# double imu_orientation_quat[4]; // w, x, y, z
# double velocity_xyz[3]; // m/s ENU
# double position_xyz[3]; // meters NED
# double pressure;

def create_fdm_packet(timestamp, altitude_m=0.0):
    """Create fdm_packet with given altitude"""
    # Calculate pressure from altitude (barometric formula)
    # P = P0 * (1 - altitude/44330)^5.255
    P0 = 101325.0  # Pa at sea level
    pressure = P0 * math.pow(1.0 - altitude_m / 44330.0, 5.255)
    
    packet = struct.pack('<' + 'd' * 18,
        timestamp,                    # timestamp
        0.0, 0.0, 0.0,               # imu_angular_velocity_rpy (no rotation)
        0.0, 0.0, -9.81,             # imu_linear_acceleration_xyz (gravity in NED)
        1.0, 0.0, 0.0, 0.0,          # imu_orientation_quat (identity quaternion)
        0.0, 0.0, 0.0,               # velocity_xyz (stationary)
        0.0, 0.0, -altitude_m,       # position_xyz (NED, so negative altitude)
        pressure                      # pressure in Pa
    )
    return packet

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("Sending fake FDM data to Betaflight SITL on port 9003")
    print("Press Ctrl+C to stop")
    print("\nSimulating altitude changes:")
    print("  0-10s:   0m (ground)")
    print("  10-20s:  10m (climbing)")
    print("  20-30s:  10m (hover)")
    print("  30-40s:  0m (descending)")
    print("  Repeats...\n")
    
    start_time = time.time()
    
    try:
        while True:
            elapsed = time.time() - start_time
            cycle_time = elapsed % 40  # 40 second cycle
            
            # Simulate altitude profile
            if cycle_time < 10:
                altitude = 0.0
            elif cycle_time < 20:
                altitude = (cycle_time - 10) * 1.0  # Climb at 1 m/s
            elif cycle_time < 30:
                altitude = 10.0
            else:
                altitude = 10.0 - (cycle_time - 30) * 1.0  # Descend at 1 m/s
            
            packet = create_fdm_packet(time.time(), altitude)
            sock.sendto(packet, ('127.0.0.1', 9003))
            
            print(f"\rTime: {elapsed:5.1f}s, Altitude: {altitude:5.1f}m, Pressure: {pressure:.0f} Pa", end='')
            time.sleep(0.01)  # 100 Hz
            
    except KeyboardInterrupt:
        print("\n\nStopped")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
