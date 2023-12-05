#include "slung_pose_estimation/State.h"
#include <iostream>
#include <kalmanif/kalmanif.h>

int main() {
    // Create a State object
    // State state;

    // // Set a specific quaternion for the State's attitude
    // // Example: Quaternion representing a 45-degree rotation around the Z axis
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, M_PI / 4); // Roll, Pitch, Yaw in radians
    // state.setAtt(quat);

    // // Retrieve the yaw, pitch, and roll values using getAttYPR
    // double yaw, pitch, roll;
    // state.getAttYPR(yaw, pitch, roll);

    // // Output the results
    // std::cout << "pi/4, 0, 0" << std::endl;
    // std::cout << "Yaw: " << yaw << std::endl;
    // std::cout << "Pitch: " << pitch << std::endl;
    // std::cout << "Roll: " << roll << std::endl;


    // // TRY ANOTHER QUATERNION
    // quat.setRPY(M_PI / 2, 0, M_PI / 4); // Roll, Pitch, Yaw in radians
    // state.setAtt(quat);
    // state.getAttYPR(yaw, pitch, roll);

    // // Output the results
    // std::cout << "" << std::endl;
    // std::cout << "pi/4, 0, pi/2" << std::endl;
    // std::cout << "Yaw: " << yaw << std::endl;
    // std::cout << "Pitch: " << pitch << std::endl;
    // std::cout << "Roll: " << roll << std::endl;

    // DO SOME KALMAN FILTER STUFF HERE!!!!

    

    return 0;
}