#pragma once

#include "MerweScaledSigmaPoints.hpp"
#include "GaussianDistribution.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace icarus
{
    template<typename T, size_t N>
    struct UnscentedKalmanFilter
    {
        explicit UnscentedKalmanFilter() :
            mSigmaPoints(0.1f)
        {
            mState.mean.setZero();
            mState.covariance.setZero();
            mState.covariance.diagonal().setConstant(0.1);
        }

        template<typename ProcessModel, typename MeasurementModel, size_t S>
        void filter(ProcessModel const & processModel, MeasurementModel const & measurementModel, GaussianDistribution<T, S> const & measurement, T timeStep)
        {
            auto points = mSigmaPoints(mState);

            for (auto & point : points) {
                point = processModel(point, timeStep);
            }

            mState = mSigmaPoints.unscentedTransform(points);
            mState.covariance += processModel.noise();

            std::array<Eigen::Matrix<T, S, 1>, 2 * N + 1> measurementPoints;

            for (int i = 0; i < 2 * N + 1; ++i) {
                measurementPoints[i] = measurementModel(points[i]);
            }

            auto measurementDistribution = mSigmaPoints.unscentedTransform(measurementPoints);
            measurementDistribution.covariance += measurement.covariance;

            Eigen::Matrix<T, N, S> gain;
            gain.setZero();

            for (int i = 0; i < mSigmaPoints.size(); ++i) {
                auto weight = mSigmaPoints.covarianceWeight(i);
                auto stateDifference = points[i] - mState.mean;
                auto measurementDifference = measurementPoints[i] - measurementDistribution.mean;

                gain += weight * stateDifference * measurementDifference.transpose();
            }

            gain *= measurementDistribution.covariance.inverse();
            // measurementDistribution.covariance.llt().solve(gain);
            // measurementDistribution.covariance.template triangularView<Eigen::Lower>().template solveInPlace<Eigen::OnTheRight>(gain);

            // auto residual = (measurement.mean - measurementDistribution.mean).eval();
            mState.mean += gain * (measurement.mean - measurementDistribution.mean);
            mState.covariance -= gain * measurementDistribution.covariance * gain.transpose();
        }

        template<typename State>
        State & state()
        {
            static_assert(sizeof(State) == sizeof(mState.mean), "State has different size than state vector.");
            return reinterpret_cast<State &>(mState.mean);
        }

        Eigen::Matrix<T, N, 1> & stateVector()
        {
            return mState.mean;
        }
    private:
        MerweScaledSigmaPoints<T, N> mSigmaPoints;
        GaussianDistribution<T, N> mState;
    };
}
