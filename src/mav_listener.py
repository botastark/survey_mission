#!/usr/bin/env python
# doent work =< because it causes mavros runing launch to close as interupts est-d mavlink connection by mavros 
import rospy
from std_msgs.msg import Bool
from pymavlink import mavutil
rospy.init_node("camera_trigger_listener")

image_save_pub = rospy.Publisher('/save_image', Bool, queue_size=1)

mavlink_connection = mavutil.mavlink_connection('/dev/ttyTHS0', baud = 921600)

def listen_for_mavlink_command():
	while not rospy.is_shutdown():
		msg = mavlink_connection.recv_match(blocking=True)
		if msg and msg.get_type() == "COMMAND_LONG":
			if msg.command == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
				rospy.loginfo("Image capture received!")
				image_save_pub.publish(True)


if __name__=="__main__":
	try:
		listen_for_mavlink_command()
	except rospy.ROSInterruptException:
		pass
