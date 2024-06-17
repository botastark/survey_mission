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
    // Constants
    const double R = 6371000.0;  // Earth radius in meters
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

// Function to read waypoints from a file and add current GPS altitude
std::vector<GPSPosition> readWaypointsFromFile(const std::string& filename, double currentGpsAltitude) {
    std::vector<GPSPosition> waypoints;
    std::ifstream infile(filename);

    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return waypoints;
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

        // Add current GPS altitude to the read altitude
        alt += currentGpsAltitude;

        // Add the waypoint to the vector
        waypoints.push_back({lat, lon, alt});
    }

    infile.close();
    return waypoints;
}
