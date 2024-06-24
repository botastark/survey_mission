#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <GeographicLib/Geoid.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <iomanip>


geographic_msgs::GeoPoseStamped current_gps;
mavros_msgs::State current_state;
// Calculate waypoints local->global
const double EARTH_RADIUS = 6378137.0;  // in meters (WGS-84 Earth radius)
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
std::ofstream log_file;
std::ofstream log_file_mission;

struct GPSPosition {
    double latitude;   // in degrees
    double longitude;  // in degrees
    double altitude;   // in meters
};
GeographicLib::Geoid _egm96("egm96-5");
