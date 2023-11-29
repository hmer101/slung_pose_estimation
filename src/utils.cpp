#include "slung_pose_estimation/utils.h"

namespace utils {
    // TRANSFORMS
    Eigen::Vector3d transform_position(const Eigen::Vector3d& p_BA, const Eigen::Vector3d& p_CB, const tf2::Quaternion& q_CB) {
        tf2::Quaternion p_BA_quat(0, p_BA.x(), p_BA.y(), p_BA.z());
        tf2::Quaternion p_BA_rotated = q_CB * p_BA_quat * q_CB.inverse();
        Eigen::Vector3d p_CA = Eigen::Vector3d(p_BA_rotated.x(), p_BA_rotated.y(), p_BA_rotated.z()) + p_CB;
        return p_CA;
    }

    tf2::Quaternion transform_orientation(const tf2::Quaternion& q_BA, const tf2::Quaternion& q_CB) {
        tf2::Quaternion q_CA = q_CB * q_BA;
        return q_CA;
    }

    std::shared_ptr<State> transform_frames(const State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger) {
        std::shared_ptr<State> state2 = std::make_shared<State>(frame2_name, CS_type::ENU);

        // Find the transform
        geometry_msgs::msg::TransformStamped tf_f1_rel_f2;
        try {
            tf_f1_rel_f2 = tf_buffer.lookupTransform(frame2_name, state.getFrame(), tf2::TimePointZero);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(logger, "Failed to find transform: %s", ex.what());
            return nullptr;
        }

        // Collect transformation vector and quaternion
        Eigen::Vector3d p_f2f1(tf_f1_rel_f2.transform.translation.x,
                            tf_f1_rel_f2.transform.translation.y,
                            tf_f1_rel_f2.transform.translation.z);

        tf2::Quaternion q_f2f1(tf_f1_rel_f2.transform.rotation.x,
                            tf_f1_rel_f2.transform.rotation.y,
                            tf_f1_rel_f2.transform.rotation.z,
                            tf_f1_rel_f2.transform.rotation.w);

        // Perform transform
        state2->setPos(transform_position(state.getPos(), p_f2f1, q_f2f1));
        state2->setAtt(transform_orientation(state.getAtt(), q_f2f1));

        return state2;
    }

    // STRING HANDLING
    int extract_id_from_name(const std::string& input) {
        size_t underscorePos = input.rfind('_');

        if (underscorePos != std::string::npos) {
            std::string lastNumber = input.substr(underscorePos + 1);
            return std::stoi(lastNumber);
        } else {
            return -1;
        }
    }
    
    // CONVERSIONS
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const State& state) {
        geometry_msgs::msg::Pose pose_msg;

        // Set position
        pose_msg.position.x = state.getPos().x();
        pose_msg.position.y = state.getPos().y();
        pose_msg.position.z = state.getPos().z();

        // Set orientation
        pose_msg.orientation.x = state.getAtt().x();
        pose_msg.orientation.y = state.getAtt().y();
        pose_msg.orientation.z = state.getAtt().z();
        pose_msg.orientation.w = state.getAtt().w();

        return pose_msg;
    }


} // namespace utils