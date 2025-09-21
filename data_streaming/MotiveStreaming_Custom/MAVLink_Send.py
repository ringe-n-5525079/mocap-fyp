#=============================================================================
# OptiTrack -> PX4 MAVLink Odometry Bridge
#=============================================================================

import sys
import time
import math
from NatNetClient import NatNetClient
from pymavlink import mavutil
import numpy as np

#=============================================================================
# CONFIGURATION
#=============================================================================
PX4_SERIAL_PORT = 'COM3'  # XBee serial port to TELEM2
PX4_BAUDRATE = 115200

CONVERT_TO_NED = True         # True if OptiTrack uses Z-up, right-hand coord
TRACK_RIGID_BODY_ID = 1       # Which rigid body to track
SEND_ODOMETRY = True          # Send ODOMETRY instead of VISION_POSITION_ESTIMATE

#=============================================================================
# GLOBALS
#=============================================================================
mav_serial = None
prev_position = None
prev_timestamp = None

#=============================================================================
# MAVLink Helper Functions
#=============================================================================
def init_mavlink_connection():
    global mav_serial
    print(f"Connecting to PX4 on {PX4_SERIAL_PORT} at {PX4_BAUDRATE} baud...")
    mav_serial = mavutil.mavlink_connection(PX4_SERIAL_PORT, baud=PX4_BAUDRATE)
    print("Waiting for PX4 heartbeat...")
    mav_serial.wait_heartbeat()
    print("Heartbeat received! PX4 connected.")

def quat_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    q = [qx, qy, qz, qw]
    Returns (roll, pitch, yaw) in radians
    """
    qx, qy, qz, qw = q
    # Roll
    sinr_cosp = 2 * (qw*qx + qy*qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2 * (qw*qy - qz*qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    # Yaw
    siny_cosp = 2 * (qw*qz + qx*qy)
    cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def send_odometry_to_px4(position, orientation, velocity=None):
    """
    Send position, orientation, and velocity to PX4 as ODOMETRY MAVLink message.
    All units in meters, m/s, radians.
    """
    global mav_serial
    timestamp_us = int(time.time() * 1e6)

    if velocity is None:
        vx = vy = vz = 0.0
    else:
        vx, vy, vz = velocity

    roll, pitch, yaw = orientation

    if CONVERT_TO_NED:
        # Convert OptiTrack Z-up right-hand to PX4 NED
        px =  position[0]
        py = -position[1]
        pz = -position[2]
        vx =  vx
        vy = -vy
        vz = -vz
        roll =  roll
        pitch = -pitch
        yaw = -yaw
    else:
        px, py, pz = position

    mav_serial.mav.odometry_send(
        timestamp_us,               # uint64_t time_usec
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame_id
        px, py, pz,                 # position
        vx, vy, vz,                 # linear velocity
        0, 0, 0,                    # angular velocity (optional, set to 0)
        roll, pitch, yaw,           # orientation
        0xFF                        # pose_covariance_unknown
    )

#=============================================================================
# OptiTrack Callback
#=============================================================================
def receive_rigid_body_frame(new_id, position, rotation):
    global prev_position, prev_timestamp

    if TRACK_RIGID_BODY_ID is not None and new_id != TRACK_RIGID_BODY_ID:
        return  # ignore other rigid bodies

    # Compute velocity if previous position exists
    now = time.time()
    velocity = None
    if prev_position is not None and prev_timestamp is not None:
        dt = now - prev_timestamp
        if dt > 0:
            velocity = [(position[i] - prev_position[i]) / dt for i in range(3)]
    prev_position = position
    prev_timestamp = now

    # Convert quaternion rotation to Euler angles
    orientation = quat_to_euler(rotation)

    try:
        send_odometry_to_px4(position, orientation, velocity)
        print(f"Sent to PX4: pos=({position[0]:.3f},{position[1]:.3f},{position[2]:.3f}) "
              f"vel=({0 if velocity is None else velocity[0]:.3f},"
              f"{0 if velocity is None else velocity[1]:.3f},"
              f"{0 if velocity is None else velocity[2]:.3f}) "
              f"yaw={orientation[2]:.3f}")
    except Exception as e:
        print(f"Error sending MAVLink to PX4: {e}")

#=============================================================================
# OptiTrack Client Setup
#=============================================================================
def setup_natnet_client(server_ip="127.0.0.1", client_ip="127.0.0.1", use_multicast=False):
    client = NatNetClient()
    client.set_client_address(client_ip)
    client.set_server_address(server_ip)
    client.set_use_multicast(use_multicast)
    client.rigid_body_listener = receive_rigid_body_frame
    return client

#=============================================================================
# MAIN
#=============================================================================
if __name__ == "__main__":
    init_mavlink_connection()

    print("Connecting to OptiTrack server...")
    use_multicast_input = input("Use multicast? (y/n) [n]: ").strip().lower()
    use_multicast = True if use_multicast_input == 'y' else False

    server_ip = input("OptiTrack Server IP [127.0.0.1]: ").strip() or "127.0.0.1"
    client_ip = input("Local Client IP [127.0.0.1]: ").strip() or "127.0.0.1"

    natnet_client = setup_natnet_client(server_ip=server_ip, client_ip=client_ip, use_multicast=use_multicast)

    print("Starting OptiTrack streaming...")
    if not natnet_client.run('d'):  # data stream
        print("Failed to start NatNet client. Exiting.")
        sys.exit(1)

    print("Running. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")
        natnet_client.shutdown()
        if mav_serial:
            mav_serial.close()
        print("Closed connections. Exiting.")
