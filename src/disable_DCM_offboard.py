#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
import subprocess

# Global variable to track the previous state
previous_mode = ""

def state_callback(state_msg):
    global previous_mode

    # Retrieve the current mode from the message
    current_mode = state_msg.mode

    # Check if the mode has changed
    if current_mode != previous_mode:
        rospy.loginfo(f"Mode changed to {current_mode}")

        # Perform actions based on the current mode
        if current_mode == "OFFBOARD":
            rospy.loginfo("Disabling drone camera manager...")
            stop_drone_manager()
        else:
            rospy.loginfo("Enabling drone camera manager...")
            start_drone_manager()

        # Update previous_mode to the current mode
        previous_mode = current_mode

def stop_drone_manager():
    # Use subprocess to run the command to stop drone_manager.service
    try:
        subprocess.run(["sudo", "systemctl", "stop", "dronecode-camera-manager.service"])
        rospy.loginfo("drone_manager.service stopped successfully.")
    except Exception as e:
        rospy.logerr(f"Error stopping drone_manager.service: {str(e)}")

def start_drone_manager():
    # Use subprocess to run the command to start drone_manager.service
    try:
        subprocess.run(["sudo", "systemctl", "start", "drone_manager.service"])
        rospy.loginfo("drone_manager.service started successfully.")
    except Exception as e:
        rospy.logerr(f"Error starting drone_manager.service: {str(e)}")

def main():
    rospy.init_node('disable_dcm_node', anonymous=True)
    
    # Subscribe to mavros/state topic
    rospy.Subscriber("/mavros/state", State, state_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

