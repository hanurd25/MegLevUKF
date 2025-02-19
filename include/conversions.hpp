//
// Created by hanur on 18.02.2025.
//

#ifndef UNSENTENCEDKALMAN_CONVERSIONS_H
#define UNSENTENCEDKALMAN_CONVERSIONS_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>


struct conversions {
   static std::tuple<double, double, double> conversions::cart2pol(double x, double y, double z);

// Convert Polar to Cartesian
   static std::tuple<double, double, double> conversions::pol2cart(double phi, double rho, double z);

//Matrix initialization: https://stackoverflow.com/questions/31549398/c-eigen-initialize-static-matrix
// roll, pitch and yaw matrices
   static Eigen::Matrix3d rot(double rotX, double rotY, double rotZ);
};


#endif //UNSENTENCEDKALMAN_CONVERSIONS_H
