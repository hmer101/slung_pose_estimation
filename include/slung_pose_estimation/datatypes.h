#ifndef DATATYPES_H
#define DATATYPES_H

#include <vector>

// Struct to store state data
struct StateData {
    std::vector<float> x, theta, theta_dot;
};


#endif // DATATYPES_H