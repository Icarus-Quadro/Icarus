#pragma once

#include <Eigen/Dense>

namespace icarus
{
    template<typename T, size_t N>
    struct GaussianDistribution
    {
        Eigen::Matrix<T, N, 1> mean;
        Eigen::Matrix<T, N, N> covariance;
    };
}
