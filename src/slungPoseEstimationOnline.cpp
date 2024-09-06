#include <rclcpp/rclcpp.hpp>

#include "slung_pose_estimation/slungPoseEstimationOnline.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

#include <chrono> // Include for std::chrono
#include <iomanip> // Include for std::put_time

SlungPoseEstimationOnline::SlungPoseEstimationOnline() : Node("slung_pose_estimation", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    // this->ns_ = this->get_namespace();
    // this->drone_id_ = utils::extract_id_from_name(this->ns_);

    this->declare_parameter<std::string>("env", "phys");
    this->get_parameter("env", this->env_);

    this->declare_parameter<int>("num_drones", 3);
    this->get_parameter("num_drones", this->num_drones_);

    this->declare_parameter<int>("first_drone_num_", 1);
    this->get_parameter("first_drone_num_", this->first_drone_num_);

    this->declare_parameter<int>("load_id", 1);
    this->get_parameter("load_id", this->load_id_);

    float estimation_timer_period;
    this->declare_parameter<double>("timer_estimation", 0.1);
    this->get_parameter("timer_estimation", estimation_timer_period);

    this->declare_parameter<double>("est_threshold_ang_dist_", 10.0);
    this->get_parameter("est_threshold_ang_dist_", this->est_threshold_ang_dist_);

    this->declare_parameter<double>("est_threshold_time_", 3.0);
    this->get_parameter("est_threshold_time_", this->est_threshold_time_);

    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto init_time = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&init_time), "%Y_%m_%d_%H_%M_%S_"); // Format the time

    // Set the logging file path
    // std::string package_share_directory = ament_index_cpp::get_package_share_directory("slung_pose_estimation");
    // std::string filename = "measurement_drone" + std::to_string(this->drone_id_) + ".txt";
    // std::string filepath = "/data/" + ss.str() + filename; // Prepend the formatted time to the filename
    // this->logging_file_path_ = package_share_directory + filepath;
 
    // VARIABLES
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    int est_timer_period_ms = static_cast<int>(estimation_timer_period * 1000); 
    this->estimation_timer_ = this->create_wall_timer(
                                std::chrono::milliseconds(est_timer_period_ms),
                                std::bind(&SlungPoseEstimationOnline::clbk_estimation, this));

    this->marker_pose_measurements_.resize(this->num_drones_);
    this->state_current_estimate_ = droneState::State("world", droneState::CS_type::ENU);

    // this->drone_phases_.resize(this->num_drones_);
    // this->sub_phase_drones_.resize(this->num_drones_);
    
    // Flags
    // this->flag_in_mission_phase_ = false;
    // this->flag_expected_pose_measurement_set_ = false;

    // ROS2
    //rclcpp::QoS qos_profile_drone_system = rclcpp::SensorDataQoS();

    // SUBSCRIBERS
    // Loop to create subscriptions for multiple drones 
    // for (int i = this->first_drone_num_; i < this->num_drones_ + this->first_drone_num_; ++i) {
    //     int drone_index = i - this->first_drone_num_;
    //     auto topic_name = "/px4_" + std::to_string(i) + "/out/current_phase";

    //     // Create subscription and bind it with a lambda
    //     this->sub_phase_drones_[drone_index] = this->create_subscription<multi_drone_slung_load_interfaces::msg::Phase>(
    //         topic_name,
    //         qos_profile_drone_system,
    //         [this, drone_index](const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg) {
    //             this->clbk_update_drone_phase(msg, drone_index);
    //         }
    //     );
    // }

    // PUBLISHERS
    // this->pub_marker_rel_camera_ = this->create_publisher<geometry_msgs::msg::Pose>
    //     this->ns_ + "/out/marker_rel_camera", qos_profile_drone_system);


    // SETUP
    this->start_time_ = this->get_clock()->now();

    // Print info
    RCLCPP_INFO(this->get_logger(), "ESTIMATION NODE %d", this->load_id_);
    
}

void SlungPoseEstimationOnline::clbk_estimation(){
    // Store best and most recent measurements to use as the estimated load pose
    int best_ind = -1;
    float best_err = std::numeric_limits<float>::infinity();
    rclcpp::Duration best_time_elapsed = rclcpp::Duration(std::numeric_limits<int32_t>::max(), 999999999);

    //int most_recent_ind = -1;
    float most_recent_err = std::numeric_limits<float>::infinity();
    rclcpp::Duration most_recent_time_elapsed = rclcpp::Duration(std::numeric_limits<int32_t>::max(), 999999999);

    int smallest_err_ind = -1;
    float smallest_err = std::numeric_limits<float>::infinity();
    rclcpp::Duration smallest_err_time_elapsed = rclcpp::Duration(std::numeric_limits<int32_t>::max(), 999999999);

    // Lookup the latest measurements from each camera
    for (int i = 0; i < this->num_drones_; ++i){
        this->marker_pose_measurements_[i] = utils::lookup_tf("world", "load_marker" + std::to_string(this->load_id_) + "_measured" + std::to_string(i+this->first_drone_num_), *this->tf_buffer_, rclcpp::Time(0), this->get_logger());
        
        // Skip this iteration if the lookup cannot be found
        if(!this->marker_pose_measurements_[i]){
            //RCLCPP_WARN(this->get_logger(), "");
            continue;
        }
        
        // Select the measurement with the least error that is within the time bounds // Select the most recent measurement that is within the error bounds
        geometry_msgs::msg::TransformStamped marker_pose_measurement_i = this->marker_pose_measurements_[i].value();  // optional_transform.value();
        droneState::State state_measured_i = utils::convert_tf_stamped_msg_to_state(marker_pose_measurement_i, "world", droneState::CS_type::ENU);
        
        float err_ang = 0.0;
        auto time_elapsed = this->get_clock()->now() - marker_pose_measurement_i.header.stamp; //std::chrono::system_clock::now()

        if(this->state_estimate_set_){
            err_ang = state_measured_i.distAngGeo(this->state_current_estimate_)*180.0 / M_PI;
        }

        // Update most recent (and perhaps best)
        if(best_ind == -1 || time_elapsed < best_time_elapsed){
            // A valid measurement to update the pose estimate
            // if(time_elapsed.seconds() < this->est_threshold_time_ && err_ang < this->est_threshold_ang_dist_){
            //     best_ind = i;
            //     //best_err = err_ang;
            //     best_time_elapsed = time_elapsed;    
            // }
            
            // Measurement may or may not be valid, but it is the most recent (store as backup)
            if(time_elapsed < most_recent_time_elapsed){ 
                //most_recent_ind = i;
                most_recent_err = err_ang;
                most_recent_time_elapsed = time_elapsed;
            }
        }

        // Update smallest error (and perhaps best)
        if(best_ind == -1 || err_ang < best_err){
            // A valid measurement to update the pose estimate
            if(time_elapsed.seconds() < this->est_threshold_time_ && err_ang < this->est_threshold_ang_dist_){
                best_ind = i;
                best_err = err_ang;
                best_time_elapsed = time_elapsed;    
            }
            
            // Measurement may or may not be valid, but it has the smallest error (store as backup)
            if(err_ang < smallest_err){ 
                smallest_err_ind = i;
                smallest_err = err_ang;
                smallest_err_time_elapsed = time_elapsed;
            }
        }
        
    }
    // All measurements are too old or too far from the previous; select the smallest error or most recent if available
    // Most recent
    // if(best_ind == -1){
    //     if(most_recent_ind == -1){ // No measurements are found
    //         RCLCPP_INFO(this->get_logger(), "No load measurements are available.");
    //         return;
    //     }
    //     else
    //     {
    //         best_ind = most_recent_ind; //most_recent_ind;
    //         RCLCPP_WARN(this->get_logger(), "Measurements exceed thresholds. Selecting the most recent with err_ang: %.2f, time_elapsed: %.2f.", most_recent_err, most_recent_time_elapsed.seconds());
    //     }
    // }

    // Smallest error
    if(best_ind == -1){
        if(smallest_err_ind == -1){ // No measurements are found
            RCLCPP_INFO(this->get_logger(), "No load measurements are available.");
            return;
        }
        else
        {
            best_ind = smallest_err_ind; //most_recent_ind;
            RCLCPP_WARN(this->get_logger(), "Measurements exceed thresholds. Selecting the smallest error measurement with err_ang: %.2f, time_elapsed: %.2f.", most_recent_err, most_recent_time_elapsed.seconds());
        }
    }

    // Extract best pose estimation
    geometry_msgs::msg::TransformStamped marker_pose_measurement_i = this->marker_pose_measurements_[best_ind].value(); 

    // Publish pose estimation
    geometry_msgs::msg::Vector3 translation = marker_pose_measurement_i.transform.translation;
    geometry_msgs::msg::Quaternion rot_q = marker_pose_measurement_i.transform.rotation;

    Eigen::Vector3d t_marker_rel_world_estimated = Eigen::Vector3d(translation.x, translation.y, translation.z);
    Eigen::Quaterniond R_marker_rel_world_measured_q = Eigen::Quaterniond(rot_q.w, rot_q.x, rot_q.y, rot_q.z);

    utils::broadcast_tf(this->get_clock()->now(),
                        "world",
                        "load_marker" + std::to_string(this->load_id_) + "_e",
                        t_marker_rel_world_estimated,
                        R_marker_rel_world_measured_q,
                        *this->tf_broadcaster_);

    // Update the stored estimation
    this->state_current_estimate_ = utils::convert_tf_stamped_msg_to_state(marker_pose_measurement_i, "world", droneState::CS_type::ENU);
    
    if(!this->state_estimate_set_){
        this->state_estimate_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Estimator initialized.");
    }
}   


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlungPoseEstimationOnline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}