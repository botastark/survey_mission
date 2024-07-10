#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/SetMavFrame.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/Geoid.hpp>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
std::string altitude_mode = "rel_alt";                                                          //"rel_alt" and "terrain_alt" "int"                                                  //"rel_alt" and "terrain_alt" "int"
std::string waypoint_filename = "/home/uvify/catkin_ws/src/survey_mission/path/waypoints.txt";  // File containing waypoints
std::string log_folder_base = "/home/uvify/catkin_ws/src/survey_mission/logs/";
std::string tol_filename = "/home/uvify/catkin_ws/src/survey_mission/path/tolerances.txt";

// std::string img_metadata_filename = "/home/uvify/Desktop/offboard_images/metadata.txt";
mavros_msgs::Altitude altitude;
geographic_msgs::GeoPoseStamped current_gps;
mavros_msgs::State current_state;
// Calculate waypoints local->global
const double EARTH_RADIUS = 6378137.0;  // in meters (WGS-84 Earth radius)
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
constexpr uint8_t FRAME_GLOBAL_TERRAIN_ALT = 10;

struct GPSPosition {
    double latitude;   // in degrees
    double longitude;  // in degrees
    double altitude;   // in meters
};

GeographicLib::Geoid _egm96("egm96-5");
