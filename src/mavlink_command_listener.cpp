#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <std_msgs/Bool.h>
// capable of capturing mavlink messages, therefore has potential to be extended further. 
// TODO find a way to test trigger!, For now acc to .plan upload from QGC to PX4, camera command is 2000

const int MAVLINK_MSG_ID_CAMERA_TRIGGER = 203;
bool capture = false;
void mavlinkMessageCallback(const mavros_msgs::Mavlink::ConstPtr& msg){
	//if (msg->msgid == MAVLINK_MSG_ID_CAMERA_TRIGGER){ // MAVLINK_MSG_ID_CAMERA_TRIGGER ???
	//	ROS_INFO("IMAGE CAPTURE TRIGGERED YEAHHH!");
	//}
	//ROS_INFO_STREAM("msg id "<<msg->msgid);
	if (msg->msgid == 203){
		ROS_INFO("MAV_CMD_DO_DIGICAM_CONTROL (203)!");
	}
	if (msg->msgid == 2000){
		ROS_INFO("MAV_CMD_IMAGE_START_CAPTURE 2000!");
		capture = true;
	}
	if (msg->msgid == 200){
		ROS_INFO("MAV_CMD_DO_CONTROL_VIDEO (200)!");
	}
	if (msg->msgid == 112){
		ROS_INFO("CAMERA_TRIGGER (112)!");
	}
	if (msg->msgid == 260){
		ROS_INFO("CAMERA_SETTINGS (260)!");
	}

}




int main(int argc, char **argv){
	ros::init(argc, argv, "mavlink_listener");
	ros::NodeHandle nh;
	ros::Subscriber mavlink_sub = nh.subscribe<mavros_msgs::Mavlink>("/mavlink/from", 10, mavlinkMessageCallback);
    	ros::Publisher trigger_pub = nh.advertise<std_msgs::Bool>("save_image", 10);

	ros::Rate loop_rate(1); // 1hz
    	std_msgs::Bool msg;
    	msg.data = false;

	while (ros::ok()) {
		msg.data = capture;
		if (msg.data==true){
			ROS_INFO("Publishing save image trigger");			
		
			trigger_pub.publish(msg);
		}		
		//ros::Duration(1.0).sleep(); //each 1 secs keep node active but do not publish
		ros::spinOnce();
		loop_rate.sleep();

	}


	ros::spin();
	return 0;
}

