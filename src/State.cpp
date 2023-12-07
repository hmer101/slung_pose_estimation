#include "slung_pose_estimation/State.h"

namespace droneState{
    // Constructor implementation
    State::State(std::string frame, CS_type cs_type, Eigen::Vector3d pos, tf2::Quaternion att, Eigen::Vector3d vel)
        : frame(frame), cs_type(cs_type), pos(pos), att(att), vel(vel) {}

    // Operator == implementation
    bool State::operator==(const State& other) const {
        return frame == other.frame && cs_type == other.cs_type &&
            pos.isApprox(other.pos) && att == other.att &&
            vel.isApprox(other.vel);
    }

    // Operator - implementation. 
    State State::operator-(const State& other) const {
        // Note that subtractions can only be performed on states with the same frame and CS type
        if (frame != other.frame || cs_type != other.cs_type) {
            throw std::invalid_argument("Subtraction can only be performed on states with the same frame and CS type");
        }else {
            return State(frame, cs_type, pos - other.pos, other.att*att.inverse(), vel - other.vel);
        }
        
    }

    // Copy method implementation
    State State::copy() const {
        return State(frame, cs_type, pos, att, vel);
    }

    // to_string method implementation
    std::string State::to_string() const {
        std::ostringstream ss;
        ss << "Frame: " << frame << ", CS type: " << static_cast<int>(cs_type)
        << ", pos: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]"
        << ", att_q: [" << att.w() << ", " << att.x() << ", " << att.y() << ", " << att.z() << "]"
        << ", vel: [" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]";
        return ss.str();
    }
} // namespace droneState