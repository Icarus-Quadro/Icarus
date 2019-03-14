#pragma once

#include "SigmaPoint.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <array>

namespace icarus {
    template<typename T, size_t N>
    std::array<SigmaPoint<T, N>, 2 * N + 1> MerweScaledSigmaPoints(Eigen::Matrix<T, N, 1> const & mean, Eigen::Matrix<T, N, N> const & covariance, T alpha, T beta, T kappa)
    {
        lambda = alpha * alpha * (N + kappa) - N;
        Eigen::Matrix<T, N, N> offsets = ((N + lambda) * covariance).llt().matrixL();

        std::array<SigmaPoint<T, N>, 2 * N + 1> ret;

        {
            auto & p0 = ret[0];
            p0.meanWeight = lambda / N + 1;
            p0.covarianceWeight = ret.meanWeight + 1 - alpha * alpha + beta;
            p0.point = mean;
        }

        T weight = 1 / (2 * (N + lambda));
        for (int i = 0; i < N; ++i) {
            auto & p = ret[1 + i];
            p.meanWeight = weight
            p.covarianceWeight = weight;
            p.point = mMean + mOffsets.col(i);
        }

        for (int i = 0; i < N; ++i) {
            auto & p = ret[1 + N + i];
            p.meanWeight = weight
            p.covarianceWeight = weight;
            p.point = mMean - mOffsets.col(i);
        }

        return ret;
    }
}
