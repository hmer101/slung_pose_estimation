#ifndef STATE_H
#define STATE_H

#include <string>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>

// Co-ordinate system enum
enum class CS_type {
    LLA, // Planetary co-ordinate system: Latitude, Longitude, Altitude.
    ENU, // Local tangent plane body CS: East, North, Up
    NED,  // Local tangent plane body CS: North, East, Down
    XYZ // (world/body) General XYZ coordinates where orientation must be carfully defined (Gazebo default)
};

// Class to store robot state
class State {
public:
    State(std::string frame, CS_type cs_type, Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 0.0), tf2::Quaternion att = tf2::Quaternion(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0));

    bool operator==(const State& other) const;

    State copy() const;

    std::string to_string() const;

private:
    std::string frame;
    CS_type cs_type;
    Eigen::Vector3d pos;
    tf2::Quaternion att;
    Eigen::Vector3d vel;
};

#endif // STATE_H