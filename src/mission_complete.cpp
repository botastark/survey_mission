#include "util.cpp"

mavros_msgs::Altitude altitude;
std::string altitude_mode = "rel_alt";  //"rel_alt" and "terrain_alt" "int"

std_msgs::Bool reached_target;
geometry_msgs::Vector3 current_target_global;
Logger *logger;  // Pointer to Logger instance

double overall_tolerance = 0.40;
double xy_tolerance = 0.20;
double h_tolerance = 0.25;

float home_asml_alt = 0.0;
bool current_gps_received = false;
// Callback functions
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps.pose.position.altitude =
        ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}
void altCallback(const mavros_msgs::Altitude::ConstPtr &msg) {
    altitude = *msg;
}
void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void targetCallback(const mavros_msgs::GlobalPositionTarget::ConstPtr &msg) {
    mavros_msgs::GlobalPositionTarget waypoint = *msg;
    current_target_global.x = waypoint.latitude;   // Update latitude
    current_target_global.y = waypoint.longitude;  // Update longitude
    current_target_global.z = waypoint.altitude;   // Update altitude
}

std_msgs::Bool missionComplete() {
    double vert_dist = haversine(current_target_global.x, current_target_global.y, current_gps.pose.position.latitude, current_gps.pose.position.longitude);
    //"rel_alt" and "terrain_alt" "int"
    float current_alt = 0.0;
    float target_alt = 0.0;
    if (altitude_mode == "int") {  // current - asml && target - asml
        current_alt = current_gps.pose.position.altitude;
	//current_alt = altitude.asml;
        target_alt = current_target_global.z;
    } else if (altitude_mode == "rel_alt") {  // current - rel && target - rel (add home alt)
        current_alt = altitude.relative;
        target_alt = current_target_global.z;

    } else {  // current - rel && target - rel
        current_alt = altitude.terrain;
        target_alt = current_target_global.z;
    }
    double hori_dist = current_alt - target_alt;
    double dist = sqrt(vert_dist * vert_dist + hori_dist * hori_dist);
    logger->logMessage("alt: " + to_string_with_precision(current_alt) + " | " + to_string_with_precision(target_alt));
    logger->logMessage("lat: " + to_string_with_precision(current_gps.pose.position.latitude) + " | " + to_string_with_precision(current_target_global.x));
    logger->logMessage("lon: " + to_string_with_precision(current_gps.pose.position.longitude) + " | " + to_string_with_precision(current_target_global.y));
    logger->logMessage("Total: " + std::to_string(dist) + " gps: " + std::to_string(vert_dist) + " h: " + std::to_string(hori_dist));
    logger->logMessage("______________________________________________________");
    if (dist < overall_tolerance && vert_dist < xy_tolerance && hori_dist < h_tolerance && dist != 0) {
        reached_target.data = true;
        ROS_INFO_ONCE("Reached waypoint!");
        logger->logMessage("reached");
    } else {
        reached_target.data = false;
    }
    return reached_target;
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);
    std::string log_folder;

    log_folder = createLogFolder();
    Logger mission_logger(log_folder, "mission_checker");
    logger = &mission_logger;
    logger->logMessage("Logger initialized.");

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Subscriber alt_sub = nh_.subscribe<mavros_msgs::Altitude>("mavros/altitude", 10, altCallback);
    ros::Publisher reached_target_pub = nh_.advertise<std_msgs::Bool>("reached_target", 10);
    ros::Subscriber target_pos_sub = nh_.subscribe<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10, targetCallback);
    ros::Rate rate(20.0);

    // Read tolerances from file
    std::ifstream tol_file("/home/uvify/catkin_ws/src/survey_mission/path/tolerances.txt");
    if (tol_file.is_open()) {
        tol_file >> overall_tolerance >> xy_tolerance >> h_tolerance;
        tol_file.close();
    } else {
        ROS_WARN("Failed to open tolerances file. Using default values.");
    }
    // Construct log message using std::string
    std::string log_msg = "Tolerances used: overall_tolerance=" + std::to_string(overall_tolerance) + ", xy_tolerance=" + std::to_string(xy_tolerance) + ", h_tolerance=" + std::to_string(h_tolerance);

    ROS_INFO_STREAM(log_msg);
    // Write message to log file
    logger->logMessage(log_msg);
    if (altitude_mode == "int") {  // current - asml && target - asml
        logger->logMessage("Altitude: absolute asml");
    } else if (altitude_mode == "rel_alt") {  // current - asml && target - rel (add home alt)
        logger->logMessage("Altitude: Relative to home position");
    } else {  // current - rel && target - rel
        logger->logMessage("Altitude: Above terrain");
    }

    logger->logMessage("gps | target");

    while (ros::ok() && !current_gps_received && home_asml_alt == 0.0) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        logger->logMessageOnce("Waiting for GPS signal...");
        home_asml_alt = current_gps.pose.position.altitude;
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    while (ros::ok()) {
        reached_target_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
