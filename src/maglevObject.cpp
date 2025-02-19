//
// Created by hanur on 18.02.2025.
//

#include "../include/maglevObject.hpp"



MaglevObject::MaglevObject() {

    magnet.r = 0.025;
    magnet.l = 0.005;
    magnet.J = 1.0;
    magnet.mass = 0.060;
    magnet.n = 100;
    magnet.I << (magnet.mass / 12) * (3 * magnet.r * magnet.r + magnet.l * magnet.l),
            (magnet.mass / 12) * (3 * magnet.r * magnet.r + magnet.l * magnet.l),
            (1.0 / 2) * magnet.mass * magnet.r * magnet.r;


    dynamics.A = Eigen::MatrixXd::Zero(12, 12);
    dynamics.A.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

    dynamics.B = Eigen::MatrixXd::Zero(12, 6);
    dynamics.B.block<6, 6>(6, 0) = Eigen::MatrixXd::Identity(6, 6);


    physical.g = 9.81;
    physical.mu0 = 4 * M_PI * 1e-7;


    solenoids.x.resize(4);
    solenoids.y.resize(4);
    solenoids.r.resize(4);
    solenoids.l.resize(4);
    solenoids.z.resize(4);

    solenoids.x << 0.02, -0.02, 0.0, 0.0;
    solenoids.y << 0.0, 0.0, 0.02, -0.02;
    solenoids.r.setConstant(0.0185 / 2);
    solenoids.l.setConstant(0.012);
    solenoids.z = solenoids.l / 2;
    solenoids.nw = 480;
    solenoids.J = 1.1;


    permanent.x.resize(4);
    permanent.y.resize(4);
    permanent.r.resize(4);
    permanent.l.resize(4);
    permanent.z.resize(4);

    permanent.x << 0.028, -0.028, 0.028, -0.028;
    permanent.y << 0.028, 0.028, -0.028, -0.028;
    //should i transpose this or not??
    permanent.r = Eigen::VectorXd::Constant(4, 0.01);
    permanent.l = Eigen::VectorXd::Constant(4, 0.009);
    permanent.z = permanent.l / 2 + Eigen::VectorXd::Constant(4, 0.0001);;
    permanent.J = 1.1;


    sensors.x.resize(1);
    sensors.y.resize(1);
    sensors.z.resize(1);

    sensors.x << 0;
    sensors.y << 0;
    sensors.z << 0;
}
