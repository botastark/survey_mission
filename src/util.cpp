#include "mission.h"
// Need to WGS84->amsl of gps altitude
//"When controlling the FCU using global setpoints, you specify the altitude as
// meters above mean sea level (AMSL). But when sensing the global position, the
// altitude reported by ~global_position/global is specified as meters above the
// WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of
// meters apart."
// https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level

double calc_geoid_height(double lat, double lon) { return _egm96(lat, lon); }
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
    return amsl +
           GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon,
                                double ellipsoid_height) {
    return ellipsoid_height +
           GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    // Convert latitude and longitude from degrees to radians
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    // Haversine formula
    double dlat = (lat2 - lat1) * M_PI / 360.0;
    double dlon = (lon2 - lon1) * M_PI / 360.0;
    double a = std::sin(dlat) * std::sin(dlat) +
               std::cos(lat1_rad) * std::cos(lat2_rad) * std::sin(dlon) *
                   std::sin(dlon);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = R * c;
    return distance;
}

// Function to read waypoints from a file
// Function to read waypoints from a file and add current GPS altitude
std::vector<GPSPosition> readWaypointsFromFile(const std::string &filename, double currentGpsAltitude, std::string altitude_mode) {
    std::vector<GPSPosition> waypoints;
    std::ifstream infile(filename);
    float offset = 0.0;
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return waypoints;
    }
    if (altitude_mode == "int") {
        offset = currentGpsAltitude;  // asml
    }

    std::string line;
    while (std::getline(infile, line)) {
        // Remove brackets and commas
        line.erase(std::remove(line.begin(), line.end(), '['), line.end());
        line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
        line.erase(std::remove(line.begin(), line.end(), ','), line.end());

        // Create a stringstream to parse the line
        std::stringstream ss(line);
        double lat, lon, alt;
        ss >> lat >> lon >> alt;

        alt += offset;
        waypoints.push_back({lat, lon, alt});
    }

    infile.close();
    return waypoints;
}
// http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/GlobalPositionTarget.html
mavros_msgs::GlobalPositionTarget create_pose(double latitude,
                                              double longitude,
                                              double altitude,
                                              std::string altitude_mode) {
    mavros_msgs::GlobalPositionTarget setpoint;
    setpoint.latitude = latitude;
    setpoint.longitude = longitude;
    setpoint.altitude = altitude;
    if (altitude_mode == "int") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    } else if (altitude_mode == "rel_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    } else if (altitude_mode == "terrain_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_TERRAIN_ALT;
    } else {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    }

    setpoint.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                         mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                         mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                         mavros_msgs::GlobalPositionTarget::FORCE |
                         mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE |
                         mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
    return setpoint;
}

void armDrone(ros::ServiceClient &arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
    }
}

void setMode(ros::ServiceClient &set_mode_client, const std::string &mode) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;
    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
        ROS_INFO_STREAM("Mode set to: " << mode);
    } else {
        ROS_ERROR_STREAM("Failed to set mode: " << mode);
    }
}

#include <signal.h>

bool g_shutdown_requested = false;

void sigintHandler(int sig) {
    g_shutdown_requested = true;
    ROS_INFO("Shutting down...");
    ros::shutdown();  // Initiate ROS shutdown
}

class Logger {
   public:
    Logger(const std::string &folder_name, const std::string &file_name_prefix) {
        initLogFile(folder_name, file_name_prefix);
    }

    ~Logger() {
        closeLogFile();
    }
    enum LogLevel {
        INFO,
        WARN,
        ERROR
    };

    void logMessage(const std::string &message, LogLevel level = INFO, bool log_once = false) {
        std::ostringstream oss;
        // oss << "[" << ros::Time::now().toSec << "] " << std::fixed << std::setprecision(15) << message;  // Set precision to 6 decimal places
        std::string log_level_prefix;
        if (g_shutdown_requested) {
            return;  // If shutdown requested, do not log further
        }
        switch (level) {
            case INFO:
                log_level_prefix = "INFO";
                break;
            case WARN:
                log_level_prefix = "WARN";
                break;
            case ERROR:
                log_level_prefix = "ERROR";
                break;
            default:
                log_level_prefix = "INFO";  // Default to INFO level if unspecified
                break;
        }

        oss << log_level_prefix << " [" << std::fixed << std::setprecision(9) << ros::Time::now().toSec() << "]: " << std::fixed << std::setprecision(15) << message;

        // Log to file
        if (log_file.is_open()) {
            log_file << oss.str() << std::endl;
            log_file.flush();  // Ensure data is written immediately
        } else {
            ROS_WARN("Log file is not open, message not logged to file: %s", message.c_str());
        }
    }
    void logMessageOnce(const std::string &message, LogLevel level = INFO) {
        if (g_shutdown_requested || logged_messages.count(message) > 0) {
            return;  // If shutdown requested or message already logged, do not log further
        }

        logged_messages.insert(message);
        logMessage(message, level);
    }

   private:
    std::ofstream log_file;
    std::set<std::string> logged_messages;

    void initLogFile(const std::string &folder_name, const std::string &file_name_prefix) {
        std::stringstream log_file_name;
        log_file_name << folder_name << "/" << file_name_prefix << ".txt";
        log_file.open(log_file_name.str());
        if (!log_file.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_file_name.str().c_str());
        } else {
            ROS_INFO("Logging to file: %s", log_file_name.str().c_str());
        }
    }

    void closeLogFile() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }
};

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();

    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H-%M", &now_tm);
    std::stringstream ss;
    ss << buffer;
    return ss.str();
}

void ensureDirectoryExists(const std::string &path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        if (mkdir(path.c_str(), 0777) != 0) {
            ROS_ERROR("Failed to create directory: %s", path.c_str());
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
    }
}

std::string createLogFolder() {
    std::string folder_name = "/home/uvify/catkin_ws/src/survey_mission/logs/";

    std::time_t rawtime = std::time(nullptr);
    struct std::tm *timeinfo = std::localtime(&rawtime);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
    folder_name += buffer;
    ensureDirectoryExists(folder_name);

    std::string folder_path = folder_name + "/" + getCurrentDateTime();
    ensureDirectoryExists(folder_path);

    return folder_path;
}

std::string to_string_with_precision(double value, int precision = 14) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}
