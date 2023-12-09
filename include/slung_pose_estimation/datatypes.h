#ifndef DATATYPES_H
#define DATATYPES_H

//#include <vector>
#include <Eigen/Dense>

// Struct to store state data
struct StateData {
    //std::vector<float> x, theta, theta_dot;
    Eigen::Vector3d x, x_dot, theta, theta_dot;
};


#endif // DATATYPES_H