//
// Created by hanur on 18.02.2025.
//

#ifndef UNSENTENCEDKALMAN_INTEGRATION_HPP
#define UNSENTENCEDKALMAN_INTEGRATION_HPP

#include <iostream>
#include <Eigen/Dense>


class integration {
    static double trapeziodal(const Eigen::VectorXd &x, const Eigen::VectorXd &y);
};

#endif //UNSENTENCEDKALMAN_INTEGRATION_HPP
