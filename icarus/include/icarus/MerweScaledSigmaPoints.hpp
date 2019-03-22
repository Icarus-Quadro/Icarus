#pragma once

#include "SigmaPoints.hpp"
#include "GaussianDistribution.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <array>

namespace icarus {
    template<typename T, size_t N>
    SigmaPoints<T, N, 2 * N + 1> MerweScaledSigmaPoints(GaussianDistribution<T, N> const & distribution, T alpha, T beta = 2, T kappa = 3 - T(N))
    {
        auto lambda = alpha * alpha * (T(N) + kappa) - T(N);
        Eigen::Matrix<T, N, N> offsets = ((T(N) + lambda) * distribution.covariance).llt().matrixL();

        SigmaPoints<T, N, 2 * N + 1> ret;

        ret.meanWeights[0] = lambda / (T(N) + lambda);
        ret.covarianceWeights[0] = ret.meanWeights[0] + 1 - alpha * alpha + beta;
        ret.points[0] = distribution.mean;

        T weight = T(1) / (2 * (T(N) + lambda));
        for (int i = 1; i < 2 * N + 1; ++i) {
            ret.meanWeights[i] = weight;
            ret.covarianceWeights[i] = weight;
        }

        for (int i = 0; i < N; ++i) {
            ret.points[1 + 2 * i + 0] = distribution.mean + offsets.col(i);
            ret.points[1 + 2 * i + 1] = distribution.mean - offsets.col(i);
        }

        return ret;
    }
}
