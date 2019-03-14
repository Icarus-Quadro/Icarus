#pragma once

#include <Eigen/Dense>

namespace icarus {
    template<typename T, size_t N>
    struct SigmaPoint
    {
        T meanWeight;
        T covarianceWeight
        Eigen::Matrix<T, N, 1> point;
    };
}