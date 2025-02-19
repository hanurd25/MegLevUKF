#include "MagLevUKF.hpp"



MagLevUKF::MagLevUKF(MaglevObject& system)
        : maglev(system), stopThread(false),
          x_est(Eigen::VectorXd::Zero(12)),
          P_est(Eigen::MatrixXd::Identity(12, 12) * 1e-2),
          X(Eigen::MatrixXd::Zero(12, 25)),
          Wm(Eigen::VectorXd::Constant(25, 1.0 / 25.0)),
          Wc(Eigen::VectorXd::Constant(25, 1.0 / 25.0)),
          L(x_est.size()),
          alpha(1e-3),
          beta(2),
          kappa(0),
          lambda(alpha * alpha * (L + kappa) - L),
          gamma(sqrt(L + lambda))  {
    loadParams();
    //load the system matrices and such
}


MagLevUKF::~MagLevUKF() {
    stopThreadSafe();
}

void MagLevUKF::loadParams() {
    double a = 0.0005, b = (2 * M_PI) / 180;
    double c = 100 * a, d = 100 * b;

    Eigen::VectorXd diag_P1(12);
    diag_P1 << a, a, a, b, b, b, c, c, c, d, d, d;
    Q = diag_P1.asDiagonal().inverse();

    double sigma1 = (1.0 / 1000) * sqrt(0.0084);
    double sigma2 = (1.0 / 1000) * sqrt(0.0085);
    double sigma3 = (1.0 / 1000) * sqrt(0.0124);

    Eigen::VectorXd diag_P2(9);
    diag_P2 << sigma1, sigma2, sigma3, sigma1, sigma2, sigma3, sigma1, sigma2, sigma3;
    R = diag_P2.asDiagonal();

    P0 = Eigen::MatrixXd::Identity(12, 12) * 1e-2;

    A = Eigen::MatrixXd::Zero(12, 12);
    A.block<6,6>(0,6) = Eigen::MatrixXd::Identity(6,6);

    B = Eigen::MatrixXd::Zero(12, 6);
    B.block<6,6>(6,0) = Eigen::MatrixXd::Identity(6,6);

    x0 = Eigen::VectorXd::Zero(12);
    x0(2) = 0.1;

    dt = 0.01;
}


void MagLevUKF::generateSigmaPoints(Eigen::MatrixXd& X, Eigen::VectorXd& Wm, Eigen::VectorXd& Wc) {

    Eigen::MatrixXd S = P_est.llt().matrixL();

    X.col(0) = x_est;
    for (int i = 0; i < L; ++i) {
        X.col(i + 1) = x_est + gamma * S.col(i);
        X.col(i + L + 1) = x_est - gamma * S.col(i);
    }

    Wm(0) = lambda / (L + lambda);
    Wc(0) = lambda / (L + lambda) + (1 - alpha * alpha + beta);
    for (int i = 1; i < 2 * L + 1; ++i) {
        Wm(i) = Wc(i) = 1 / (2 * (L + lambda));
    }
}

void MagLevUKF::startThread() {
    ukfThread = std::jthread([this] { runUKFLoop(); });
}

void MagLevUKF::stopThreadSafe() {
    stopThread.store(true);
    if (ukfThread.joinable()) {
        ukfThread.join();
    }
}

void MagLevUKF::runUKFLoop() {
    while (!stopThread.load()) {
        std::lock_guard<std::mutex> lock(ukfMutex);

        Eigen::VectorXd u = Eigen::VectorXd::Zero(4);
        u(2) = 9.81;

        //assigning new valies to the state vecot Wm and Wc
        generateSigmaPoints(X, Wm, Wc);

        Eigen::MatrixXd X_pred(12, 25);
        for (int i = 0; i < 25; ++i) {
            // 0.01 is dt
            X_pred.col(i) = MaglevSystemDynamics::maglevSysDynamics(X.col(i), maglev, u) * 0.01 + X.col(i);
        }


        //initialize this shit in MagLev UKF instead??
        Eigen::VectorXd x_pred = X_pred * Wm;
        Eigen::MatrixXd P_pred = Q;

        for (int i = 0; i < 25; ++i) {
            //gonna make the same for Y?!?!?
            P_pred += Wc(i) * (X_pred.col(i) - x_pred) * (X_pred.col(i) - x_pred).transpose();
        }

        //Receiving the measurements form serial communication
        //from teensy 4.1
        Eigen::VectorXd y_meas = getMeasurements(x_pred);
        x_est = x_pred;
        P_est = P_pred;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Prevent CPU overload
    }
}