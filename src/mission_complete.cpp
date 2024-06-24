#include "util.cpp"


std_msgs::Bool reached_target;
geometry_msgs::Vector3 current_target_global;

double tolerance = 0.20;

std::string getCurrentTimeStr() {
    std::ostringstream ss;
    ros::Time now = ros::Time::now();
    ss << std::fixed << std::setprecision(2) << now.toSec();
    return ss.str();
}

void logWithTime(const std::string &message) {
    log_file << "[" << getCurrentTimeStr() << "] " << message << std::endl;
}


// Callback functions
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr &msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x = waypoint.pose.position.latitude;  // Update latitude
    current_target_global.y = waypoint.pose.position.longitude;  // Update longitude
    current_target_global.z = waypoint.pose.position.altitude;  // Update altitude
}

std_msgs::Bool missionComplete() {

    double vert_dist = haversine(current_target_global.x, current_target_global.y,
                  current_gps.pose.position.latitude,
                  current_gps.pose.position.longitude);
    double hori_dist = current_gps.pose.position.altitude - current_target_global.z;
    double dist = sqrt(vert_dist * vert_dist + hori_dist * hori_dist);


    logWithTime("gps h: " + std::to_string(current_gps.pose.position.altitude) + " target h: " + std::to_string(current_target_global.z));
    logWithTime("gps lat: " + std::to_string(current_gps.pose.position.latitude) + " target lat: " + std::to_string(current_target_global.x));
    logWithTime("gps lon: " + std::to_string(current_gps.pose.position.longitude) + " target lon: " + std::to_string(current_target_global.y));
    logWithTime("dist: " + std::to_string(dist) + " vert_dist: " + std::to_string(vert_dist) + " hori_dist: " + std::to_string(hori_dist));


    // Log the data to the file

    
    if (dist < tolerance && vert_dist < 0.11 && hori_dist < 0.2 && dist != 0) {
        reached_target.data = true;
        ROS_INFO("Reached waypoint!");
        logWithTime("reached");
    } else {
        reached_target.data = false;
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


    // Create a log file with the start time in its name
    std::time_t now = std::time(0);
    std::tm *ltm = std::localtime(&now);
    std::ostringstream log_filename;
    log_filename << "/home/uvify/catkin_ws/src/survey_mission/logs/missioncheck_log_"
                 << 1900 + ltm->tm_year
                 << std::setw(2) << std::setfill('0') << 1 + ltm->tm_mon
                 << std::setw(2) << std::setfill('0') << ltm->tm_mday << "_"
                 << std::setw(2) << std::setfill('0') << ltm->tm_hour
                 << std::setw(2) << std::setfill('0') << ltm->tm_min
                 << std::setw(2) << std::setfill('0') << ltm->tm_sec << ".txt";
    
    log_file.open(log_filename.str());
    log_file.open("/home/uvify/catkin_ws/src/survey_mission/mission_log.txt");

    // Check if the log file is open
    if (!log_file.is_open()) {
        ROS_ERROR("Failed to open log file.");
        return 1;
    }

    while (ros::ok()) {
        reached_target_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    log_file.close();
    return 0;
}
