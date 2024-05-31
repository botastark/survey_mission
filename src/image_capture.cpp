#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>

std::string gstreamer_pipeline(int sensor_id = 0,
                               int capture_width = 1920,
                               int capture_height = 1080,
                               int display_width = 960,
                               int display_height = 540,
                               int framerate = 30,
                               int flip_method = 0) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! "
           "video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! "
           "nvvidconv flip-method=" + std::to_string(flip_method) + " ! "
           "video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" + std::to_string(display_height) + ", format=(string)BGRx ! "
           "videoconvert ! "
           "video/x-raw, format=(string)BGR ! appsink";
}

cv::VideoCapture video_capture;
std::string base_path = "/home/uvify/Desktop/survey/";
std::string original_filename = "captured_image";

void saveImageCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        cv::Mat frame;
        video_capture >> frame;
        if (!frame.empty()) {

            ros::Time timestamp = ros::Time::now();
            std::stringstream ss;
            ss << timestamp.sec << "_" << timestamp.nsec;
            std::string filename = base_path + original_filename + "_" + ss.str() + ".jpg";
            
            cv::imwrite(filename, frame);
            // ROS_INFO("Image captured: %s", filename.c_str());
        } else {
            ROS_ERROR("Capture read error");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_capture_node");
    ros::NodeHandle nh;

    video_capture.open(gstreamer_pipeline());

    if (!video_capture.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }
    ros::Subscriber sub = nh.subscribe("save_image", 20, saveImageCallback);
    cv::Mat frame;

    while (ros::ok()) {
    
        video_capture >> frame;
    
        ros::spinOnce();
    }

    ros::spin();
    video_capture.release();


    return 0;
}

