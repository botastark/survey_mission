#!/usr/bin/env python
# needs to be checked as idk if mavros/cmd/command is a valid rostopic. doesn't show up in rostopic list and cant' be triggered from QGC console ))
import rospy
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandLong

rospy.init_node("camera_trigger_listener")

image_save_pub = rospy.Publisher('/save_image', Bool, queue_size=1)

def listen_for_mavlink_command(msg):
	if msg.command == 200:
		rospy.loginfo("Image capture received!")
		image_save_pub.publish(True)

def main():
	rospy.Subscriber("/mavros/cmd/command", CommandLong, listen_for_mavlink_command);
	rospy.spin()


if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
