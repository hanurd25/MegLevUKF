#ifndef UNSENTENCEDKALMAN_MAGLEVSYSTEMDYNAMICS_HPP
#define UNSENTENCEDKALMAN_MAGLEVSYSTEMDYNAMICS_HPP

#include "../include/maglevObject.hpp"

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif


class MaglevSystemDynamics {
public:
    MaglevSystemDynamics(const Eigen::Vector3d& inertia);

    static Eigen::VectorXd computeForceAndTorque(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const MaglevObject& params);
    static Eigen::VectorXd maglevSysDynamics(const Eigen::VectorXd& x, const MaglevObject& params, const Eigen::VectorXd& u);
    static Eigen::Vector3d computeFieldBase(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z,
                                     const Eigen::VectorXd& u, const MaglevObject& params);
    static std::tuple<double, double, double> computeFieldCircularWireCartesian(double x, double y, double z, double rIndex, double uIndex, double mu0);
    static std::tuple<double, double, double> computeFieldCircularWirePolar(double phi, double rho, double z, double r, double I, double mu0);

private:
    double mass;
    Eigen::Matrix3d inertiaMatrix;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
};

#endif //UNSENTENCEDKALMAN_MAGLEVSYSTEMDYNAMICS_HPP
