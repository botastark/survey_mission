#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_trigger_publisher");
    ros::NodeHandle nh;

    ros::Publisher trigger_pub = nh.advertise<std_msgs::Bool>("save_image", 10);

    ros::Rate loop_rate(1); // 1hz
    std_msgs::Bool msg;
    msg.data = true;

    while (ros::ok()) {
        ROS_INFO("Publishing save image trigger");
        trigger_pub.publish(msg);
	ros::Duration(10.0).sleep(); //each 10 secs keep node active but do not publish
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

