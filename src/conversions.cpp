//
// Created by hanur on 18.02.2025.
//

#include "conversions.hpp"


std::tuple<double, double, double> conversions::cart2pol(double x, double y, double z) {
    //double rho = std::sqrt(x * x + y * y);
    //double phi = std::atan2(y, x);
    return {std::atan2(y, x), std::sqrt(x * x + y * y), z};
}

// Convert Polar to Cartesian
std::tuple<double, double, double> conversions::pol2cart(double phi, double rho, double z) {
    //double x = rho * std::cos(phi);
    //double y = rho * std::sin(phi);
    return {rho * std::cos(phi), rho * std::sin(phi), z};
}

//Matrix initialization: https://stackoverflow.com/questions/31549398/c-eigen-initialize-static-matrix
// roll, pitch and yaw matrices
Eigen::Matrix3d conversions::rot(double rotX, double rotY, double rotZ) {
    return (Eigen::Matrix3d() << cos(rotZ), -sin(rotZ), 0,
            sin(rotZ), cos(rotZ), 0,
            0, 0, 1).finished()
           * (Eigen::Matrix3d() << cos(rotY), 0, sin(rotY),
            0, 1, 0,
            -sin(rotY), 0, cos(rotY)).finished()
           * (Eigen::Matrix3d() << 1, 0, 0,
            0, cos(rotX), -sin(rotX),
            0, sin(rotX), cos(rotX)).finished();
}

