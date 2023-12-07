#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "slung_pose_estimation/datatypes.h" 
#include "slung_pose_estimation/utils.h"


int main() {
    // Get the share directory for your package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("slung_pose_estimation");

    // Construct the full path to the data file
    std::string data_file_path = package_share_directory + "/data/data_vnav_0.01.txt";

    std::cout << "Data file path: " << data_file_path << std::endl;

    float t;
    //std::vector<float> xl, theta_l, T;
    StateData load_data;
    std::vector<StateData> tension_data;
    std::vector<StateData> drones_data;

    std::ifstream file(data_file_path);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return false;
    }

    int cnt = 0;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string segment;
        
        // Read time
        std::getline(iss, segment, ' ');
        t = std::stof(segment);

        std::cout << "t: " << t << std::endl;

        // Read load state
        // x
        std::getline(iss, segment, ' ');
        load_data.x = utils::splitAndConvert(segment, ',');

        std::cout << "load_x : " << load_data.x[0] << " " << load_data.x[1] << " " << load_data.x[2] << std::endl;

        // x_dot and x_ddot
        std::getline(iss, segment, ' ');
        std::getline(iss, segment, ' '); 
        
        // theta
        std::getline(iss, segment, ' ');
        load_data.theta = utils::splitAndConvert(segment, ',');

        std::cout << "load_theta : " << load_data.theta[0] << " " << load_data.theta[1] << " " << load_data.theta[2] << std::endl;

        // theta_dot
        std::getline(iss, segment, ' ');
        load_data.theta_dot = utils::splitAndConvert(segment, ',');

        std::cout << "load_theta_dot : " << load_data.theta_dot[0] << " " << load_data.theta_dot[1] << " " << load_data.theta_dot[2] << std::endl;

        // theta_ddot
        std::getline(iss, segment, ' ');


        // Read tension vectors
        for (int i = 0; i < 3; i++)
        {
            std::getline(iss, segment, ' ');
            StateData tension;
            tension.x = utils::splitAndConvert(segment, ',');
            tension_data.push_back(tension);

            std::cout << "tension: " << tension_data[i].x[0] << " " << tension_data[i].x[1] << " " << tension_data[i].x[2] << std::endl;
        }

        // Read drone states
        for (int i = 0; i < 3; i++)
        {   
            // x
            std::getline(iss, segment, ' ');
            StateData drone;
            drone.x = utils::splitAndConvert(segment, ',');
            
            // x_dot and x_ddot
            std::getline(iss, segment, ' ');
            std::getline(iss, segment, ' ');

            // theta
            std::getline(iss, segment, ' ');
            drone.theta = utils::splitAndConvert(segment, ',');

            // theta_dot and theta_ddot
            std::getline(iss, segment, ' ');
            std::getline(iss, segment, ' ');

            // Store drone data
            drones_data.push_back(drone);
            std::cout << "drone: " << drones_data[i].x[0] << " " << drones_data[i].x[1] << " " << drones_data[i].x[2] << std::endl;
            std::cout << "drone_theta: " << drones_data[i].theta[0] << " " << drones_data[i].theta[1] << " " << drones_data[i].theta[2] << std::endl;
        }

        if(cnt == 10){
            return 0;
        }

        cnt++;
    }

    return 0;
}