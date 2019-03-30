#pragma once

#include "SigmaPoints.hpp"
#include "GaussianDistribution.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <array>

namespace icarus {
    template<typename T, size_t N>
    struct MerweScaledSigmaPoints
    {
        explicit MerweScaledSigmaPoints(T alpha, T beta = 2, T kappa = 3 - T(N))
        {
            mLambda = alpha * alpha * (T(N) + kappa) - T(N);
            mFirstMeanWeight = mLambda / (T(N) + mLambda);
            mCovarianceWeight = mFirstMeanWeight + 1 - alpha * alpha + beta;
            mNextWeights = T(1) / (2 * (T(N) + mLambda));
        }

        static constexpr size_t size()
        {
            return 2 * N + 1;
        }

        std::array<Eigen::Matrix<T, N, 1>, size()> operator()(GaussianDistribution<T, N> const & distribution) const
        {
            Eigen::Matrix<T, N, N> offsets = ((T(N) + mLambda) * distribution.covariance).llt().matrixL();

            std::array<Eigen::Matrix<T, N, 1>, size()> ret;
            ret[0] = distribution.mean;

            for (int i = 0; i < N; ++i) {
                ret[1 + 2 * i + 0] = distribution.mean + offsets.col(i);
                ret[1 + 2 * i + 1] = distribution.mean - offsets.col(i);
            }

            return ret;
        }

        template<int M>
        GaussianDistribution<T, M> unscentedTransform(std::array<Eigen::Matrix<T, M, 1>, 2 * N + 1> const & points) const
        {
            GaussianDistribution<T, M> ret;

            ret.mean = mFirstMeanWeight * points[0];

            for (int i = 1; i < 2 * N + 1; ++i) {
                ret.mean += mNextWeights * points[i];
            }

            auto difference = (points[0] - ret.mean).eval();
            ret.covariance.template triangularView<Eigen::Lower>() = mCovarianceWeight * difference * difference.transpose();

            for (int i = 1; i < 2 * N + 1; ++i) {
                difference = (points[i] - ret.mean).eval();
                ret.covariance.template selfadjointView<Eigen::Lower>().rankUpdate(difference, mNextWeights);
            }

            return ret;
        }

        T covarianceWeight(size_t index) const
        {
            if (index == 0) {
                return mCovarianceWeight;
            } else {
                return mNextWeights;
            }
        }
    private:
        T mLambda;
        T mFirstMeanWeight;
        T mCovarianceWeight;
        T mNextWeights;
    };
}
