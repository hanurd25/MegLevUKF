
#include "../include/integration.hpp"


double integration::trapeziodal(const Eigen::VectorXd &x, const Eigen::VectorXd &y) {
    //if (x.size() != y.size() || x.size() < 2) {
    //    throw std::invalid_argument("x and y must have the same size and at least 2 elements.");
    //}

    double integral = 0.0;
    for (int i = 0; i < x.size() - 1; ++i) {
        integral += 0.5 * (x(i + 1) - x(i)) * (y(i + 1) + y(i));
    }
    return integral;
}
}