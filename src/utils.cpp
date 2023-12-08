#include "slung_pose_estimation/utils.h"

namespace utils {
    // TRANSFORMS
    std::optional<geometry_msgs::msg::TransformStamped> lookup_tf( 
        const std::string& target_frame, 
        const std::string& source_frame,
        tf2_ros::Buffer& tfBuffer,
        const rclcpp::Time& time, 
        rclcpp::Logger logger) 
    {
        try {
            auto transform = tfBuffer.lookupTransform(target_frame, source_frame, time);
            return transform;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger, "Failed to find transform: %s", ex.what());
            return std::nullopt;
        }
    }


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

    std::shared_ptr<droneState::State> transform_frames(const droneState::State& state, const std::string& frame2_name, tf2_ros::Buffer& tf_buffer, rclcpp::Logger logger, droneState::CS_type cs_out_type) {
        std::shared_ptr<droneState::State> state2 = std::make_shared<droneState::State>(frame2_name, cs_out_type); //CS_type::ENU

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

    // MATH
    // Helper function to calculate the trace of a 3x3 matrix
    float getTrace(const tf2::Matrix3x3& matrix) {
        return matrix[0][0] + matrix[1][1] + matrix[2][2];
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

    // Helper function to split a string by a delimiter and convert to float
    std::vector<float> splitAndConvert(const std::string& s, char delimiter) {
        std::vector<float> result;
        std::istringstream ss(s);
        std::string token;

        while (std::getline(ss, token, delimiter)) {
            result.push_back(std::stof(token));
        }

        return result;
    }
    
    // CONVERSIONS
    geometry_msgs::msg::Pose convert_state_to_pose_msg(const droneState::State& state) {
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

    tf2::Quaternion convert_rvec_to_quaternion(const cv::Vec3d& rvec) {
        // Convert rotation vector to rotation matrix
        cv::Mat rotMat;
        cv::Rodrigues(rvec, rotMat);

        // Convert OpenCV matrix to tf2 matrix
        tf2::Matrix3x3 tf2_rotMat(
            rotMat.at<double>(0, 0), rotMat.at<double>(0, 1), rotMat.at<double>(0, 2),
            rotMat.at<double>(1, 0), rotMat.at<double>(1, 1), rotMat.at<double>(1, 2),
            rotMat.at<double>(2, 0), rotMat.at<double>(2, 1), rotMat.at<double>(2, 2)
        );

        // Create a quaternion from the rotation matrix
        tf2::Quaternion tf2_quaternion;
        tf2_rotMat.getRotation(tf2_quaternion);

        return tf2_quaternion;
    }


} // namespace utils