#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <GeographicLib/Geoid.hpp>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Global variable to store the log file stream
std::ofstream log_file;

geographic_msgs::GeoPoseStamped current_gps;
mavros_msgs::State current_state;
// Calculate waypoints local->global
const double EARTH_RADIUS = 6378137.0;  // in meters (WGS-84 Earth radius)
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

struct GPSPosition {
    double latitude;   // in degrees
    double longitude;  // in degrees
    double altitude;   // in meters
};
GeographicLib::Geoid _egm96("egm96-5");
