
#include "../include/maglevSystemDynamics.hpp"
#include "../include/conversions.hpp"
#include <algorithm>
#include <iostream>
#include <cmath>
#include <tuple>

MaglevSystemDynamics::MaglevSystemDynamics(const Eigen::Vector3d& inertia)
        : inertiaMatrix(inertia.asDiagonal()) {

    A = Eigen::MatrixXd::Zero(12, 12);
    A.block<6, 6>(0, 6) = Eigen::MatrixXd::Identity(6, 6);

    B = Eigen::MatrixXd::Zero(12, 6);
    B.block<6, 6>(6, 0) = Eigen::MatrixXd::Identity(6, 6);
}

Eigen::VectorXd MaglevSystemDynamics::maglevSysDynamics(const Eigen::VectorXd& x, const MaglevObject& params, const Eigen::VectorXd& u) {
    Eigen::VectorXd dx(12);

    Eigen::VectorXd f = computeForceAndTorque(x, u, params);

    Eigen::Matrix3d inertiaMatrix = params.magnet.I.asDiagonal();
    Eigen::VectorXd f_nl(6);
    f_nl.head(3) = f.head(3) / params.magnet.mass;
    f_nl.tail(3) = inertiaMatrix.inverse() * (f.tail(3) - x.segment<3>(9).cross(inertiaMatrix * x.segment<3>(9)));

    // s

    dx = params.dynamics.A * x + params.dynamics.B * f_nl;

    dx(6)  -= x(6);
    dx(7)  -= x(7);
    dx(8)  -= 10 * x(8);
    dx(9)  -= 2 * x(9);
    dx(10) -= 2 * x(10);

    dx(5)  = 0;
    dx(11) = 0;

    return dx;
}

Eigen::VectorXd MaglevSystemDynamics::computeForceAndTorque(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const MaglevObject& params) {
    Eigen::VectorXd result(6); // [fx, fy, fz, tx, ty, tz]
    result.setZero();

    double K = -params.magnet.J / params.physical.mu0;

    int n = params.magnet.n;
    Eigen::VectorXd theta = Eigen::VectorXd::LinSpaced(n, 0, 2 * M_PI - 2 * M_PI / n);
    Eigen::VectorXd px = params.magnet.r * theta.array().cos();
    Eigen::VectorXd py = params.magnet.r * theta.array().sin();
    Eigen::VectorXd pz = Eigen::VectorXd::Zero(n);

        result(0) = K * px.sum();
        result(1) = K * py.sum();
        result(2) = K * pz.sum();

    return result;
}


Eigen::Vector3d MaglevSystemDynamics::computeFieldBase(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z,
                      const Eigen::VectorXd& u, const MaglevObject& params) {
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;

    // Only doing and implementing fast mode
    for (size_t i = 0; i < params.solenoids.r.size(); ++i) {
        //[bxTemp,byTemp,bzTemp]
        auto  [bxTemp,byTemp,bzTemp] = computeFieldCircularWireCartesian(
                params.solenoids.x(i), //ok
                params.solenoids.y(i), //ok
                params.solenoids.z(i), //ok
                params.solenoids.r(i), //ok
                u(i), //ok
                params.physical.mu0 //ok
        );

        bx += bxTemp * params.solenoids.nw;
        by += byTemp * params.solenoids.nw;
        bz += bzTemp * params.solenoids.nw;
    }
    return Eigen::Vector3d(bx, by, bz);
}
//the x, y and z inputs are all singular coordinates
//[bxTemp,byTemp,bzTemp] something like that
std::tuple<double, double, double> MaglevSystemDynamics::computeFieldCircularWireCartesian(double x, double y, double z, double rIndex, double uIndex, double mu0){
    auto [phi, rho, zconv] = conversions::cart2pol(x,y,z);
    auto[bPhi,bRho,bz] = computeFieldCircularWirePolar(phi, rho, z, rIndex, uIndex, mu0);
    // returning [bx,by,bz]
    return conversions::pol2cart(bPhi,bRho,bz);
}

//I in matlab is equal to u
std::tuple<double, double, double> MaglevSystemDynamics::computeFieldCircularWirePolar(double phi, double rho, double z, double r, double I, double mu0) {
    // Tolerance for rho ~= 0
    const double tol = 1e-6;

    if (std::abs(rho) < tol) {
        return {0.0, 0.0, (mu0 * r * r * I) / (2 * std::pow(r * r + z * z, 1.5))};
    }
    double c = (mu0 * I) / (4 * M_PI * std::sqrt(r * rho));
    double k2 = (4 * r * rho) / ((r + rho) * (r + rho) + z * z);
    k2 = std::max(0.0, std::min(k2, 1.0));

    double k = std::sqrt(k2);
    double K = std::ellint_1(k, M_PI / 2); // Complete elliptic integral of the first kind
    double E = std::ellint_2(k, M_PI / 2); // Complete elliptic integral of the second kind

    // return {bphi, brho, bz};
    return {phi, -(z / rho) * c * std::sqrt(k2) *
                 (K - ((rho * rho + r * r + z * z) / ((rho - r) * (rho - r) + z * z)) * E), c * std::sqrt(k2) *
                                                                                            (K - ((rho * rho - r * r + z * z) / ((rho - r) * (rho - r) + z * z)) * E)};
}





void computeFieldCircularWirePolar(
        double phi, double rho, double z,
        double r, double I, double mu0
) {
    double c_factor = mu0 * I / (4 * M_PI);
    double k2 = (4 * r * rho) / (std::pow(r + rho, 2) + std::pow(z, 2));
    k2 = std::clamp(k2, 0.0, 1.0); // Clamping for numerical stability

    double K = std::comp_ellint_1(std::sqrt(k2));
    double E = std::comp_ellint_2(std::sqrt(k2));

    double c = c_factor / std::sqrt(r * rho);

    // Assign bPhi directly from phi
    double bPhi = phi;

    // Compute brho and bz using Engmark & Hoang's equations
    double bRho = -z / rho * c * std::sqrt(k2) *
           (K - ((std::pow(rho, 2) + r * r + std::pow(z, 2)) / (std::pow(rho - r, 2) + std::pow(z, 2))) * E);

    double bz = c * std::sqrt(k2) *
         (K - ((std::pow(rho, 2) - r * r + std::pow(z, 2)) / (std::pow(rho - r, 2) + std::pow(z, 2))) * E);

    const double tol = 1e-6;
    if (std::abs(rho) < tol) {
        bPhi = 0;
        bRho = 0;
        bz = mu0 * r * r * I / (2 * std::pow(r * r + z * z, 1.5));
    }
}