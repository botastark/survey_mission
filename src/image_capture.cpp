#include <opencv2/opencv.hpp>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v2.0/common/mavlink.h>
#include "util.cpp"
cv::VideoCapture video_capture;
cv::Mat frame;

std::string base_path = "/home/uvify/Desktop/offboard_images/";
std::string camera_param_path = "/home/uvify/catkin_ws/src/survey_mission/path/camera_params.txt";
std::string survey_folder;

bool armed_;
bool metadata_written_;
geometry_msgs::Point launch_global_position_;
geometry_msgs::Point launch_local_position_;
geometry_msgs::Quaternion launch_orientation_;
std::string camera_params_;

sensor_msgs::NavSatFix current_gps_;
nav_msgs::Odometry current_pose_;
#include <boost/make_shared.hpp>

std::string survey_folder_;
std::string img_metadata_filename;
#include <unordered_set>

// Global set to store unique command IDs
std::unordered_set<uint16_t> received_commands;
// Callback function to handle incoming MAVLink messages

std::string gstreamer_pipeline(int wbmode = 0,
                               int capture_width = 3264,
                               int capture_height = 1848,
                               int framerate = 28,
                               std::string format = "NV12",
                               int flip_method = 0,
                               double contrast = 1.0,
                               double brightness = -0.1,
                               int exposure_time_min = 13000,
                               int exposure_time_max = 683709000,
                               int gain_min = 1,
                               int gain_max = 1,
                               bool aelock = false,
                               const std::string& udp_ip = "192.168.0.107",
                               int udp_port = 5000) {
    return "nvarguscamerasrc sensor-id=0 wbmode=" + std::to_string(wbmode) + "  exposuretimerange='" + std::to_string(exposure_time_min) + " " + std::to_string(exposure_time_max) + "' gainrange='" + std::to_string(gain_min) + " " + std::to_string(gain_max) + "' aelock=" + std::to_string(aelock) +
           " ! "
           "video/x-raw(memory:NVMM), width=(int)" +
           std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) + ", format=(string)" + format + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! " +
           "nvvidconv flip-method=" + std::to_string(flip_method) + " ! " +
           "video/x-raw, format=(string)RGBA ! " +
           "videobalance contrast=" + std::to_string(contrast) + " brightness=" + std::to_string(brightness) + " ! " +
           "videoconvert ! " +
           //	"video/x-raw, format=(string)BGR ! "+
           //	"appsink";
           "tee name=t ! queue ! appsink name=appsink t. ! " +
           "queue ! videoconvert ! omxh265enc insert-vui=1 ! h265parse ! rtph265pay config-interval=1 ! udpsink host=" + udp_ip + " port=" + std::to_string(udp_port);
}
std::string quaternionToString(const geometry_msgs::Quaternion& quat) {
    std::stringstream ss;
    ss << "[" << to_string_with_precision(quat.x) << ", " << to_string_with_precision(quat.y) << ", " << to_string_with_precision(quat.z) << ", " << to_string_with_precision(quat.w) << "]";
    return ss.str();
}
std::string localPositionToString(const geometry_msgs::Point& position) {
    std::stringstream ss;
    ss << "[" << to_string_with_precision(position.x) << ", " << to_string_with_precision(position.y) << ", " << to_string_with_precision(position.z) << "]";
    return ss.str();
}
// TODO: DOUBLE CHECK IF EULER ANGLES ARE RIGHT FOR NED FRAME or we need to change it
// Function to convert quaternion to roll, pitch, and yaw in the NED frame
geometry_msgs::Point quaternionToRPY(const geometry_msgs::Quaternion& q_orient) {
    Eigen::Quaterniond quat(q_orient.w, q_orient.x, q_orient.y, q_orient.z);
    Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX convention

    geometry_msgs::Point rpy_orient;
    rpy_orient.x = euler_angles[2];  // Roll
    rpy_orient.y = euler_angles[1];  // Pitch
    rpy_orient.z = euler_angles[0];  // Yaw

    return rpy_orient;
}


void saveImageCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        video_capture >> frame;
        if (!frame.empty()) {
            std::string timestamp = getCurrentDateTime("hms-ms");
            //std::string dateFolder = base_path + getCurrentDateTime("ymd");
            //ensureDirectoryExists(dateFolder);
            std::string image_path = survey_folder + "/" + timestamp + ".png";
            cv::imwrite(image_path, frame);

            // ROS_INFO("Image captured: %s", filename.c_str());
            ImageMetadata metadata;

            metadata.timestamp = timestamp;

            geometry_msgs::Point global_position;
            global_position.x = current_gps_.latitude;
            global_position.y = current_gps_.longitude;
            global_position.z = current_gps_.altitude;
            metadata.global_position = localPositionToString(global_position);
            // store local position and orientation as strings
            geometry_msgs::Quaternion orientation = current_pose_.pose.pose.orientation;
            metadata.orientation = quaternionToString(orientation);
            metadata.local_position = localPositionToString(current_pose_.pose.pose.position);
            metadata.rpy_orientation = localPositionToString(quaternionToRPY(orientation));

            metadata.writeToTxt(img_metadata_filename);

        } else {
            ROS_ERROR("Capture read error");
        }
    }
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps_ = *msg;
    current_gps_.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_pose_ = *msg;
}

void writeLaunchInfo(const std::string& filename) {
    std::ofstream file(filename, std::ios_base::out | std::ios_base::app);  // Open in append mode
    geometry_msgs::Point rpy_orient = quaternionToRPY(launch_orientation_);

    if (file.is_open()) {
        file << "Launch Global Position (lat, lon, alt): " << localPositionToString(launch_global_position_) << std::endl;
        file.flush();
        file << "Launch Local Position (x, y, z): " << localPositionToString(launch_local_position_) << std::endl;
        file.flush();
        file << "Launch Orientation (quaternion): " << quaternionToString(launch_orientation_) << std::endl;
        file.flush();
        file << "Launch Orientation (roll-pitch-yaw): " << localPositionToString(rpy_orient) << std::endl;
        file.flush();

        file << "Camera Parameters: " << camera_params_ << std::endl;
        file.flush();
        file << "---------------------\n";
        file.flush();
        file.close();
    } else {
        std::cerr << "Failed to open file: " << filename << std::endl;
    }
}



void mavlinkCallback(const mavros_msgs::Mavlink::ConstPtr& msg)
{
    mavlink_message_t mav_msg;
    mavlink_status_t status;

    // Fill the mavlink_message_t structure
    mav_msg.msgid = msg->msgid;
    mav_msg.len = msg->len;
    mav_msg.seq = msg->seq;
    mav_msg.sysid = msg->sysid;
    mav_msg.compid = msg->compid;
    std::memcpy(mav_msg.payload64, msg->payload64.data(), msg->payload64.size() * sizeof(uint64_t));
    //if (received_commands.find(mav_msg.msgid) == received_commands.end()) {
        // If not found, add to set and process
        received_commands.insert(mav_msg.msgid);
    	if (mav_msg.msgid == MAV_CMD_IMAGE_START_CAPTURE){
        	ROS_INFO("Received MAV_CMD_IMAGE_START_CAPTURE command");
        	std_msgs::Bool msg_tosave;
    		msg_tosave.data = true;
//        	saveImageCallback(msg_tosave);
        	saveImageCallback(boost::make_shared<std_msgs::Bool>(msg_tosave));

    	}
    	else if (mav_msg.msgid == MAV_CMD_IMAGE_STOP_CAPTURE)
    	{
        	ROS_INFO("Received MAV_CMD_IMAGE_STOP_CAPTURE command");
    	}
    	else
    	{
        	ROS_INFO("Received MAVLink command with ID: %d",mav_msg.msgid );
    	}
    //}
}
void stateCallback(const mavros_msgs::StateConstPtr& msg) {
    armed_ = msg->armed;
    //armed_=true;
    if (armed_ && !metadata_written_) {
        launch_global_position_.x = current_gps_.latitude;
        launch_global_position_.y = current_gps_.longitude;
        launch_global_position_.z = current_gps_.altitude;
        launch_local_position_.x = current_pose_.pose.pose.position.x;
        launch_local_position_.y = current_pose_.pose.pose.position.y;
        launch_local_position_.z = current_pose_.pose.pose.position.z;

        launch_orientation_.x = current_pose_.pose.pose.orientation.x;
        launch_orientation_.y = current_pose_.pose.pose.orientation.y;
        launch_orientation_.z = current_pose_.pose.pose.orientation.z;
        launch_orientation_.w = current_pose_.pose.pose.orientation.w;

        writeLaunchInfo(img_metadata_filename);
        metadata_written_ = true;  // Set flag to true to indicate launch info has been written
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "image_capture_node");
    ros::NodeHandle nh_camera;
    ros::Subscriber sub = nh_camera.subscribe("/save_image", 20, saveImageCallback);
    ros::Subscriber gps_sub_ = nh_camera.subscribe("/mavros/global_position/global", 10, gpsCallback);
    ros::Subscriber pose_sub_ = nh_camera.subscribe("/mavros/local_position/odom", 10, poseCallback);
    ros::Subscriber state_sub_ = nh_camera.subscribe("/mavros/state", 10, stateCallback);
    ros::Subscriber mav_sub = nh_camera.subscribe("/mavlink/from", 10, mavlinkCallback);

    // Read configuration from text file
    std::unordered_map<std::string, std::string> config = readConfigFile(camera_param_path);

    int wbmode = std::stoi(config["wbmode"]);
    int capture_width = std::stoi(config["capture_width"]);
    int capture_height = std::stoi(config["capture_height"]);
    int framerate = std::stoi(config["framerate"]);
    std::string format = config["format"];
    int flip_method = std::stoi(config["flip_method"]);
    double contrast = std::stod(config["contrast"]);
    double brightness = std::stod(config["brightness"]);
    int exposure_time_min = std::stoi(config["exposure_time_min"]);
    int exposure_time_max = std::stoi(config["exposure_time_max"]);
    int gain_min = std::stoi(config["gain_min"]);
    int gain_max = std::stoi(config["gain_max"]);
    bool aelock = (config["aelock"] == "true");
    
    survey_folder = base_path + getCurrentDateTime("ymd");
    ensureDirectoryExists(survey_folder);
    survey_folder+="/"+getCurrentDateTime("hm");
    ensureDirectoryExists(survey_folder);
    img_metadata_filename = survey_folder + "/metadata.txt";
    

    std::string pipeline = gstreamer_pipeline(wbmode, capture_width, capture_height, framerate, format, flip_method, contrast, brightness, exposure_time_min, exposure_time_max, gain_min, gain_max, aelock);

    // std::cout << "Using pipeline: \n\t" << pipeline << "\n";
    camera_params_  = pipeline;

    video_capture.open(pipeline, cv::CAP_GSTREAMER);

    if (!video_capture.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    while (ros::ok()) {
        video_capture >> frame;
        ros::spinOnce();
    }
    ros::spin();
    video_capture.release();
    return 0;
}
