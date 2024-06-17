#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State
import subprocess

# Global variable to track the image_capture_node process
image_capture_process = None
previous_mode = ""

def state_callback(state_msg):
    global previous_mode, image_capture_process

    current_mode = state_msg.mode

    if current_mode != previous_mode:
        rospy.loginfo("Mode changed to {}".format(current_mode))

        if current_mode == "OFFBOARD":
            rospy.loginfo("Disabling drone camera manager...")
            stop_drone_manager()
            start_image_capture_node()

        else:
            rospy.loginfo("Enabling drone camera manager...")
            stop_image_capture_node()
            start_drone_manager()

        previous_mode = current_mode

def start_image_capture_node():
    global image_capture_process
    if image_capture_process is None:
        try:
            image_capture_process = subprocess.Popen(["rosrun", "survey_mission", "image_capture_node"])
            rospy.loginfo("image_capture_node started.")
        except Exception as e:
            rospy.logerr("Error starting image_capture_node: {}".format(str(e)))

def stop_image_capture_node():
    global image_capture_process
    if image_capture_process is not None:
        try:
            image_capture_process.terminate()
            rospy.loginfo("image_capture_node stopped.")
        except Exception as e:
            rospy.logerr("Error stopping image_capture_node: {}".format(str(e)))
        finally:
            image_capture_process = None

def stop_drone_manager():
    try:
        subprocess.call(["sudo", "systemctl", "stop", "dronecode-camera-manager.service"])
        rospy.loginfo("drone_manager.service stopped successfully.")
    except Exception as e:
        rospy.logerr("Error stopping drone_manager.service: {}".format(str(e)))

def start_drone_manager():
    try:
        subprocess.call(["sudo", "systemctl", "start", "dronecode-camera-manager.service"])
        rospy.loginfo("drone_manager.service started successfully.")
    except Exception as e:
        rospy.logerr("Error starting drone_manager.service: {}".format(str(e)))

def main():
    rospy.init_node('disable_dcm_node', anonymous=True)
    
    rospy.Subscriber("/mavros/state", State, state_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()

