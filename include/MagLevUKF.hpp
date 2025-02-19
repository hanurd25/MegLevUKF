
#ifndef UNSENTENCEDKALMAN_MAGLEVUKF_HPP
#define UNSENTENCEDKALMAN_MAGLEVUKF_HPP

#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <thread>
#include <mutex>
#include "maglevObject.hpp"
#include "maglevSystemDynamics.hpp"

#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif


class MagLevUKF {
public:
    explicit MagLevUKF(MaglevObject& system);
    ~MagLevUKF();

    Eigen::MatrixXd Q, R, P0;
    Eigen::MatrixXd A, B;
    Eigen::VectorXd x0;


    MaglevObject& maglev;
    std::jthread ukfThread;
    std::atomic<bool> stopThread;
    std::mutex ukfMutex;

    Eigen::VectorXd x_est;
    Eigen::MatrixXd P_est;
    Eigen::MatrixXd X;
    Eigen::VectorXd Wm, Wc;


    //these are the sigma points parameters
    int L;
    double alpha;
    double beta;
    double kappa;
    double lambda;
    double gamma;

    void loadParams();
    void generateSigmaPoints(Eigen::MatrixXd& X, Eigen::VectorXd& Wm, Eigen::VectorXd& Wc);
    void runUKFLoop();
    void startThread();

    void stopThreadSafe();


private:
    double dt;
    Eigen::VectorXd X; //state matrix

};


#endif //UNSENTENCEDKALMAN_MAGLEVUKF_HPP
