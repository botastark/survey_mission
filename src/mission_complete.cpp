#include "util.cpp"

std_msgs::Bool reached_target;
geometry_msgs::Vector3 current_target_global;
std::ofstream log_file_cher;

double overall_tolerance = 0.20; // Default value, will be updated from file
double xy_tolerance = 0.11;      // Default value, will be updated from file
double h_tolerance = 0.2;        // Default value, will be updated from file

// Callback functions
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    // current_gps = *msg;
    current_gps.pose.position.altitude =
        ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr &msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x =
        waypoint.pose.position.latitude;  // Update latitude
    current_target_global.y =
        waypoint.pose.position.longitude;  // Update longitude
    current_target_global.z =
        waypoint.pose.position.altitude;  // Update altitude
}

std_msgs::Bool missionComplete() {
    double vert_dist =
        haversine(current_target_global.x, current_target_global.y,
                  current_gps.pose.position.latitude,
                  current_gps.pose.position.longitude);
    double hori_dist =
        current_gps.pose.position.altitude - current_target_global.z;
    double dist = sqrt(vert_dist * vert_dist + hori_dist * hori_dist);
    log_file_cher << "gps h: " << current_gps.pose.position.altitude
                  << " target h: " << current_target_global.z << std::endl;

    log_file_cher << "gps lat: " << current_gps.pose.position.latitude
                  << " target lat: " << current_target_global.x << std::endl;

    log_file_cher << "gps  lon: " << current_gps.pose.position.longitude
                  << " target lon: " << current_target_global.y << std::endl;
    // Log the data to the file

    log_file_cher << "dist: " << dist << " vert_dist: " << vert_dist
                  << " hori_dist: " << hori_dist << std::endl;
    if (dist < overall_tolerance && vert_dist < xy_tolerance && hori_dist < h_tolerance && dist != 0) {
        reached_target.data = true;
        ROS_INFO("Reached waypoint!");
        log_file_cher << "reached" << std::endl;
    } else {
        reached_target.data = false;
        // ROS_INFO_STREAM("dist: " << dist << " vert_dist: " << vert_dist
        //                          << " hori_dist: " << hori_dist);
    }

    return reached_target;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    ros::Subscriber state_sub =
        nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>(
        "mavros/global_position/global", 10, gpsCallback);
    ros::Publisher reached_target_pub =
        nh_.advertise<std_msgs::Bool>("/reached_target", 10);
    ros::Subscriber target_pos_sub =
        nh_.subscribe<geographic_msgs::GeoPoseStamped>(
            "mavros/setpoint_position/global", 10, targetCallback);
    ros::Rate rate(20.0);

    // Open a log file for writing
    log_file_cher.open("/home/uvify/catkin_ws/src/survey_mission/mission_log.txt");

    // Check if the log file is open
    if (!log_file_cher.is_open()) {
        ROS_ERROR("Failed to open log file.");
        return 1;
    }
    // Read tolerances from file
    std::ifstream tol_file("/home/uvify/catkin_ws/src/survey_mission/path/tolerances.txt");
    if (tol_file.is_open()) {
        tol_file >> overall_tolerance >> xy_tolerance >> h_tolerance;
        tol_file.close();
    } else {
        ROS_WARN("Failed to open tolerances file. Using default values.");
    }

    // Construct log message using std::string
    std::string log_msg = "Tolerances used: overall_tolerance=" +
                          std::to_string(overall_tolerance) +
                          ", xy_tolerance=" + std::to_string(xy_tolerance) +
                          ", h_tolerance=" + std::to_string(h_tolerance);

    ROS_INFO_STREAM(log_msg);
    // Write message to log file
    log_file_cher << log_msg << std::endl;

    while (ros::ok()) {
        reached_target_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    log_file_cher.close();
    return 0;
}
