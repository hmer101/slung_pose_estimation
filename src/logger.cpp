#include <rclcpp/rclcpp.hpp>

#include "slung_pose_estimation/logger.h"
#include "slung_pose_estimation/utils.h"
#include "slung_pose_estimation/State.h"

#include "multi_drone_slung_load_interfaces/msg/phase.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

//#include <chrono> // Include for std::chrono
//#include <iomanip> // Include for std::put_time

Logger::Logger() : Node("logger", rclcpp::NodeOptions().use_global_arguments(true)) {
    // PARAMETERS
    this->ns_ = this->get_namespace();
    this->id_ = utils::extract_id_from_name(this->ns_);

    this->declare_parameter<std::string>("env", "phys");
    this->get_parameter("env", this->env_);

    this->declare_parameter<int>("num_drones", 3);
    this->get_parameter("num_drones", this->num_drones_);

    this->declare_parameter<int>("first_drone_num_", 1);
    this->get_parameter("first_drone_num_", this->first_drone_num_);

    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H_%M_%S_"); // Format the time

    // Set the logging file path
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("slung_pose_estimation");
    std::string filename = "logger" + std::to_string(this->id_) + ".txt";
    std::string filepath = "/data/" + ss.str() + filename; // Prepend the formatted time to the filename
    this->logging_file_path_ = package_share_directory + filepath;

    // VARIABLES
    this->states_drones_rel_world_gt.resize(this->num_drones_);
    this->states_drones_rel_world_desired.resize(this->num_drones_);

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Logger::clbk_timer, this)); //100ms

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock()); //tf2_ros::Buffer(std::make_shared<rclcpp::Clock>());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*(this->tf_buffer_));

    this->drone_phases_.resize(this->num_drones_);
    this->sub_phase_drones_.resize(this->num_drones_);
    this->flag_in_mission_phase_ = false;

    // ROS2
    rclcpp::QoS qos_profile_drone_system_ = rclcpp::SensorDataQoS();

    // SUBSCRIBERS
    // Loop to create subscriptions for multiple drones
    for (int i = this->first_drone_num_; i < this->num_drones_ + this->first_drone_num_; ++i) {
        int drone_index = i - this->first_drone_num_;
        auto topic_name = "/px4_" + std::to_string(i) + "/out/current_phase";

        // Create subscription and bind it with a lambda
        this->sub_phase_drones_[drone_index] = this->create_subscription<multi_drone_slung_load_interfaces::msg::Phase>(
            topic_name,
            qos_profile_drone_system_,
            [this, drone_index](const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg) {
                this->clbk_update_drone_phase(msg, drone_index);
            }
        );
    }

    // PUBLISHERS

    // SETUP
    this->start_time_ = this->get_clock()->now();

    // Print info
    RCLCPP_INFO(this->get_logger(), "LOGGING NODE %d", this->id_);
    
}

void Logger::clbk_update_drone_phase(const multi_drone_slung_load_interfaces::msg::Phase::SharedPtr msg, const int drone_index)
{
    this->drone_phases_[drone_index].phase = msg->phase;
}

void Logger::clbk_timer(){ 
    // LOAD
    // Get pose
    auto load_rel_world_gt = utils::lookup_tf("world","load" + std::to_string(this->id_) + "_gt", *this->tf_buffer_, rclcpp::Time(0), this->get_logger());
    auto load_rel_world_desired = utils::lookup_tf("world","load" + std::to_string(this->id_) + "_d", *this->tf_buffer_, rclcpp::Time(0), this->get_logger());

    // Check if the load poses are available, skip logging if not
    if(!load_rel_world_gt || !load_rel_world_desired){
        RCLCPP_WARN(this->get_logger(), "Load pose ground truth or desired not yet available. Skipping logging.");
        return;
    }

    // Convert to states
    droneState::State state_load_rel_world_gt = droneState::State("load" + std::to_string(this->id_) + "_gt", droneState::CS_type::ENU);
    state_load_rel_world_gt.setPos(Eigen::Vector3d(load_rel_world_gt->transform.translation.x, load_rel_world_gt->transform.translation.y, load_rel_world_gt->transform.translation.z));
    state_load_rel_world_gt.setAtt(tf2::Quaternion(load_rel_world_gt->transform.rotation.x, load_rel_world_gt->transform.rotation.y, load_rel_world_gt->transform.rotation.z, load_rel_world_gt->transform.rotation.w));

    droneState::State state_load_rel_world_desired = droneState::State("load" + std::to_string(this->id_) + "_d", droneState::CS_type::ENU);
    state_load_rel_world_desired.setPos(Eigen::Vector3d(load_rel_world_desired->transform.translation.x, load_rel_world_desired->transform.translation.y, load_rel_world_desired->transform.translation.z));
    state_load_rel_world_desired.setAtt(tf2::Quaternion(load_rel_world_desired->transform.rotation.x, load_rel_world_desired->transform.rotation.y, load_rel_world_desired->transform.rotation.z, load_rel_world_desired->transform.rotation.w));

    // DRONES
    // Loop through each drone's desired and ground truth states and log their poses
    for (int i = this->first_drone_num_; i < this->num_drones_+this->first_drone_num_; ++i) 
    {
        // Get individual drone poses
        auto drone_rel_world_gt = utils::lookup_tf("world","drone" + std::to_string(i) + "_gt", *this->tf_buffer_, rclcpp::Time(0), this->get_logger());
        auto drone_rel_world_desired = utils::lookup_tf("world","drone" + std::to_string(i) + "_d", *this->tf_buffer_, rclcpp::Time(0), this->get_logger());

        // Check if the drone poses are available, skip logging if not
        if(!drone_rel_world_gt || !drone_rel_world_desired){
            RCLCPP_WARN(this->get_logger(), "Drone %d's pose ground truth or desired not yet available. Skipping logging.", i);
            return;
        }

        // Convert to states
        droneState::State state_drone_rel_world_gt = droneState::State("drone" + std::to_string(i) + "_gt", droneState::CS_type::ENU);
        state_drone_rel_world_gt.setPos(Eigen::Vector3d(drone_rel_world_gt->transform.translation.x, drone_rel_world_gt->transform.translation.y, drone_rel_world_gt->transform.translation.z));
        state_drone_rel_world_gt.setAtt(tf2::Quaternion(drone_rel_world_gt->transform.rotation.x, drone_rel_world_gt->transform.rotation.y, drone_rel_world_gt->transform.rotation.z, drone_rel_world_gt->transform.rotation.w));

        droneState::State state_drone_rel_world_desired = droneState::State("drone" + std::to_string(i) + "_d", droneState::CS_type::ENU);
        state_drone_rel_world_desired.setPos(Eigen::Vector3d(drone_rel_world_desired->transform.translation.x, drone_rel_world_desired->transform.translation.y, drone_rel_world_desired->transform.translation.z));
        state_drone_rel_world_desired.setAtt(tf2::Quaternion(drone_rel_world_desired->transform.rotation.x, drone_rel_world_desired->transform.rotation.y, drone_rel_world_desired->transform.rotation.z, drone_rel_world_desired->transform.rotation.w));

        // Store drone states
        this->states_drones_rel_world_gt[i-this->first_drone_num_] = state_drone_rel_world_gt;
        this->states_drones_rel_world_desired[i-this->first_drone_num_] = state_drone_rel_world_desired; 
    }
    
    // DATA DIVIDERS   
    bool drones_in_mission_phase = std::all_of(this->drone_phases_.begin(), this->drone_phases_.end(), [](const multi_drone_slung_load_interfaces::msg::Phase& phase_msg) {
        return phase_msg.phase == multi_drone_slung_load_interfaces::msg::Phase::PHASE_MISSION_START; 
    });

    // Add a blank line at the start and end of the mission phase
    if((drones_in_mission_phase && !this->flag_in_mission_phase_) || (!drones_in_mission_phase && this->flag_in_mission_phase_)){       
        std::ofstream logFile;
        logFile.open(this->logging_file_path_, std::ios::app);

        if (logFile.is_open()) {
            // Log load poses
            logFile << std::endl;
        }

        this->flag_in_mission_phase_ = !this->flag_in_mission_phase_;
    }

    // Log the ground truth poses
    this->log_gt_poses(this->logging_file_path_, state_load_rel_world_gt, state_load_rel_world_desired, this->states_drones_rel_world_gt, this->states_drones_rel_world_desired);
}

void Logger::log_gt_poses(const std::string &filename, const droneState::State& state_load_rel_world_gt, const droneState::State& state_load_rel_world_desired, const std::vector<droneState::State>& states_drones_rel_world_gt, const std::vector<droneState::State>& states_drones_rel_world_desired){
    std::ofstream logFile;
    logFile.open(filename, std::ios::app); // Open in append mode

    // Log drone and load ground truth poses
    float time = this->get_clock()->now().seconds() - this->start_time_.seconds();

    // Convert load poses to Eigen
    Eigen::Vector3d pos_load_rel_world_desired = state_load_rel_world_desired.getPos();
    Eigen::Vector3d pos_load_rel_world_gt = state_load_rel_world_gt.getPos();

    Eigen::Vector3d rpy_load_rel_world_desired = state_load_rel_world_desired.getAttYPR();
    Eigen::Vector3d rpy_load_rel_world_gt = state_load_rel_world_gt.getAttYPR();
    
    // Calculate load pose errors
    Eigen::Vector3d pos_err_load = pos_load_rel_world_gt - pos_load_rel_world_desired;
    Eigen::Vector3d att_err_load = rpy_load_rel_world_gt - rpy_load_rel_world_desired;

    float distTransLoad = state_load_rel_world_gt.distTrans(state_load_rel_world_desired);
    float distAngGeoLoad = state_load_rel_world_gt.distAngGeo(state_load_rel_world_desired)*180.0 / M_PI;
    
    if (logFile.is_open()) {       
        // Log load poses
        logFile << time << " "
                << pos_load_rel_world_desired.x() << " " << pos_load_rel_world_desired.y() << " " << pos_load_rel_world_desired.z() << " "
                << rpy_load_rel_world_desired.x() << " " << rpy_load_rel_world_desired.y() << " " << rpy_load_rel_world_desired.z() << " "
                << pos_load_rel_world_gt.x() << " " << pos_load_rel_world_gt.y() << " " << pos_load_rel_world_gt.z() << " "
                << rpy_load_rel_world_gt.x() << " " << rpy_load_rel_world_gt.y() << " " << rpy_load_rel_world_gt.z() << " "
                << pos_err_load.x() << " " << pos_err_load.y() << " " << pos_err_load.z() << " "
                << att_err_load.x() << " " << att_err_load.y() << " " << att_err_load.z() << " "
                << distTransLoad << " " << distAngGeoLoad << " ";


        // Loop through each drone's desired and ground truth states and log their poses
        for (int i = 0; i < this->num_drones_; ++i) // for (auto it = states_drones_rel_world.begin(); it != end; ++it) //for (const auto &drone_rel_world : states_drones_rel_world)
        {
            // Get individual drone states
            const droneState::State state_drone_rel_world_desired = states_drones_rel_world_desired[i];
            const droneState::State state_drone_rel_world_gt = states_drones_rel_world_gt[i];

            // Convert to Eigen
            Eigen::Vector3d pos_drone_rel_world_desired = state_drone_rel_world_desired.getPos();
            Eigen::Vector3d pos_drone_rel_world_gt = state_drone_rel_world_gt.getPos();
            Eigen::Vector3d rpy_drone_rel_world_desired = state_drone_rel_world_desired.getAttYPR();
            Eigen::Vector3d rpy_drone_rel_world_gt = state_drone_rel_world_gt.getAttYPR();

            // Calculate errors
            Eigen::Vector3d pos_err_drone = pos_drone_rel_world_gt - pos_drone_rel_world_desired;
            Eigen::Vector3d att_err_drone = rpy_drone_rel_world_gt - rpy_drone_rel_world_desired;

            float distTransDrone = state_drone_rel_world_gt.distTrans(state_drone_rel_world_desired);
            float distAngGeoDrone = state_drone_rel_world_gt.distAngGeo(state_drone_rel_world_desired)*180.0 / M_PI;

            // Log drone poses
            logFile << pos_drone_rel_world_desired.x() << " " << pos_drone_rel_world_desired.y() << " " << pos_drone_rel_world_desired.z() << " ";
            logFile << rpy_drone_rel_world_desired.x() << " " << rpy_drone_rel_world_desired.y() << " " << rpy_drone_rel_world_desired.z() << " ";
            logFile << pos_drone_rel_world_gt.x() << " " << pos_drone_rel_world_gt.y() << " " << pos_drone_rel_world_gt.z() << " ";
            logFile << rpy_drone_rel_world_gt.x() << " " << rpy_drone_rel_world_gt.y() << " " << rpy_drone_rel_world_gt.z() << " ";
            logFile << pos_err_drone.x() << " " << pos_err_drone.y() << " " << pos_err_drone.z() << " ";
            logFile << att_err_drone.x() << " " << att_err_drone.y() << " " << att_err_drone.z() << " ";
            logFile << distTransDrone << " " << distAngGeoDrone;

            // Add space if drone is not the last in the list, otherwise add linebreak
            if(i < this->num_drones_ - 1){
                logFile << " ";
            }else{
                logFile << std::endl;
            }
        }

        logFile.close();
    }else {
        RCLCPP_ERROR(this->get_logger(), "Unable to open file for logging");
    }
}



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Logger>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}