#!/usr/bin/env python3
import math
import time
import rtde_control
import rtde_receive

ROBOT_IP = "192.168.8.4"

def main():
    print(f"Connecting to UR5e at {ROBOT_IP}...")

    try:
        # Interfaces for receiving data and sending commands
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        print("Connected successfully!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Read current joint angles [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
    actual_q = rtde_r.getActualQ()
    print(f"\nCurrent joint angles (radians):")
    print([round(q, 3) for q in actual_q])

    # Copy the current position so we only change what we want
    target_q = actual_q[:]

    # Add 10 degrees (in radians) to the Base joint (index 0)
    target_q[0] += math.radians(10.0)

    # Set very safe, slow kinematic limits
    velocity = 0.1      # rad/s
    acceleration = 0.1  # rad/s^2

    print("\nMoving the base joint by +10 degrees...")
    print("READY ON THE E-STOP!")
    time.sleep(2) # Give yourself 2 seconds to grab the pendant

    # Execute the movement (moveJ = Joint space move)
    # This call will block until the movement is finished
    success = rtde_c.moveJ(target_q, velocity, acceleration)

    if success:
        print("Movement complete.")
    else:
        print("Movement failed or was interrupted.")

    # Clean up
    rtde_c.stopScript()

if __name__ == '__main__':
    main()