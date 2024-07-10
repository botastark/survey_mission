#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <sys/stat.h>

#include <fstream>
#include <sstream>
#include <unordered_map>

cv::VideoCapture video_capture;
cv::Mat frame;

std::string base_path = "/home/uvify/Desktop/offboard_images/";

std::string camera_param_path = "/home/uvify/catkin_ws/src/survey_mission/path/camera_params.txt";

std::unordered_map<std::string, std::string> readConfigFile(const std::string& filename) {
	std::unordered_map<std::string, std::string> config;
	std::ifstream file(filename);
	std::string line;
	while (std::getline(file, line)) {
		std::istringstream line_stream(line);
		std::string key, value;
        	if (std::getline(line_stream, key, '=') && std::getline(line_stream, value)) {
			config[key] = value;
        	}
    	}return config;
}


std::string gstreamer_pipeline( int wbmode = 0,
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
				const std::string& udp_ip= "192.168.0.107",
                                int udp_port = 5000
				){
    return "nvarguscamerasrc sensor-id=0 wbmode="+std::to_string(wbmode) +"  exposuretimerange='" + std::to_string(exposure_time_min) + " " + std::to_string(exposure_time_max)+"' gainrange='"+std::to_string(gain_min) + " " + std::to_string(gain_max)+"' aelock=" +std::to_string(aelock) + " ! "
	"video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" + std::to_string(capture_height) + ", format=(string)" + format + ", framerate=(fraction)" + std::to_string(framerate) + "/1 ! " +
	"nvvidconv flip-method=" + std::to_string(flip_method) + " ! " +
	"video/x-raw, format=(string)RGBA ! " +
	"videobalance contrast=" + std::to_string(contrast) + " brightness=" + std::to_string(brightness) +" ! "+
	"videoconvert ! " +
//	"video/x-raw, format=(string)BGR ! "+
//	"appsink";
	"tee name=t ! queue ! appsink name=appsink t. ! " +
	"queue ! videoconvert ! omxh265enc insert-vui=1 ! h265parse ! rtph265pay config-interval=1 ! udpsink host=" + udp_ip + " port=" + std::to_string(udp_port);
}


std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    auto fractional_seconds = now - ms;

    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H-%M-%S", &now_tm);
    std::stringstream ss;
    ss << buffer << '-' << std::setfill('0') << std::setw(3) << fractional_seconds.count();
    return ss.str();
}

void ensureDirectoryExists(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        // Directory does not exist, create it
        if (mkdir(path.c_str(), 0777) != 0) {
            ROS_ERROR("Failed to create directory: %s", path.c_str());
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
    }
}



void saveImageCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        video_capture >> frame;
        if (!frame.empty()) {
	    std::string dateFolder = base_path;

            // Create directory for current date if it does not exist
            
            dateFolder += createDateFolder();

            // Check if directory exists, create if not
            ensureDirectoryExists(dateFolder);
            std::string filename = dateFolder + "/" + getCurrentDateTime() + ".png";
            
            cv::imwrite(filename, frame);
            // ROS_INFO("Image captured: %s", filename.c_str());
        } else {
            ROS_ERROR("Capture read error");
        }
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_capture_node");
	ros::NodeHandle nh_camera;
	ros::Subscriber sub = nh_camera.subscribe("/save_image", 20, saveImageCallback);

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

	std::string pipeline = gstreamer_pipeline(wbmode, capture_width, capture_height, framerate, format, flip_method, contrast, brightness, exposure_time_min, exposure_time_max, gain_min, gain_max, aelock);

	//std::string pipeline = gstreamer_pipeline();
	//std::cout << "Using pipeline: \n\t" << pipeline << "\n";

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
