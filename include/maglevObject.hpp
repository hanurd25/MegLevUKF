#ifndef UNSENTENCEDKALMAN_MAGLEVOBJECT_HPP
#define UNSENTENCEDKALMAN_MAGLEVOBJECT_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

#include <cmath>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif



struct MaglevObject {
    struct {
        double J;       // Magnetic moment
        double mass;
        double r;
        double l;
        int n;
        Eigen::Vector3d I;
    } magnet;

    struct {
        double g;
        double mu0;     // Permeability constant
    } physical;

    struct {
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;
    } dynamics;

    struct {
        Eigen::VectorXd x;
        Eigen::VectorXd y;
        Eigen::VectorXd r;
        Eigen::VectorXd l;
        Eigen::VectorXd z;
        double J;
        int nw;
    } solenoids;


    struct {
        Eigen::VectorXd x; // position
        Eigen::VectorXd y; // position
        Eigen::VectorXd r;
        Eigen::VectorXd l;
        Eigen::VectorXd z;
        double J;
    } permanent;

    struct {
        Eigen::VectorXd x;
        Eigen::VectorXd y;
        Eigen::VectorXd z;
    } sensors;

    // Constructor
    MaglevObject();
};

#endif //UNSENTENCEDKALMAN_MAGLEVOBJECT_HPP
