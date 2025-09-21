#!/usr/bin/env python3
import time
import argparse
import math
from pymavlink import mavutil
from NatNetClient import NatNetClient  # Ensure NatNet SDK Python wrapper is installed

# --- MAVLink setup ---
def connect_mavlink(conn_str, baud=115200):
    print(f"Connecting to MAVLink on {conn_str}...")
    if "udp" in conn_str:
        mav = mavutil.mavlink_connection(conn_str)
    else:
        mav = mavutil.mavlink_connection(conn_str, baud=baud)
    mav.wait_heartbeat()
    print(f"Connected to system {mav.target_system}, component {mav.target_component}")
    return mav

def send_vision_position(mav, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    usec = int(time.time() * 1e6)
    mav.mav.vision_position_estimate_send(usec, x, y, z, roll, pitch, yaw)

# --- OptiTrack callback ---
def receive_rigid_body_frame(new_id, position, rotation):
    """
    Callback from OptiTrack.
    position: (x,y,z) in meters
    rotation: quaternion (qx,qy,qz,qw) from OptiTrack
    """
    global mav, scale_factor

    if mav is None:
        return

    # Convert quaternion to Euler (roll, pitch, yaw)
    qx, qy, qz, qw = rotation
    # standard aerospace sequence: ZYX
    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(t3, t4)

    # Apply scaling if needed (OptiTrack often uses mm)
    x, y, z = [coord * scale_factor for coord in position]

    send_vision_position(mav, x, y, z, roll, pitch, yaw)

# --- Main ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OptiTrack to PX4 MAVLink bridge")
    parser.add_argument("--conn", type=str, required=True,
                        help="Connection string: udpout:127.0.0.1:14550 for SITL or COM3 for serial")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate for serial")
    parser.add_argument("--scale", type=float, default=0.001, help="Scale factor (e.g., 0.001 for mm→m)")
    parser.add_argument("--server", type=str, default="127.0.0.1", help="OptiTrack server IP")
    parser.add_argument("--local", type=str, default="127.0.0.1", help="Local IP for NatNet client")
    args = parser.parse_args()

    scale_factor = args.scale
    mav = connect_mavlink(args.conn, args.baud)

    # Setup OptiTrack client
    streaming_client = NatNetClient(server=args.server, local=args.local)
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.run()

    print("Streaming OptiTrack data → PX4...")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Exiting...")
        streaming_client.stop()
