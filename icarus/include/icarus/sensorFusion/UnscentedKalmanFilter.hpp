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
            mSigmaPoints(0.5f)
        {
            reset();
        }

        template<typename ProcessModel, typename MeasurementModel, size_t S>
        void filter(ProcessModel const & processModel, MeasurementModel const & measurementModel, GaussianDistribution<T, S> const & measurement, T timeStep)
        {
            auto points = mSigmaPoints(mState);

            for (auto & point : points) {
                point = processModel(point, timeStep);
            }

            mState = mSigmaPoints.unscentedTransform(points);
            mState.covariance.template triangularView<Eigen::Lower>() += processModel.noise();

            std::array<Eigen::Matrix<T, S, 1>, 2 * N + 1> measurementPoints;

            for (int i = 0; i < 2 * N + 1; ++i) {
                measurementPoints[i] = measurementModel(points[i]);
            }

            auto measurementDistribution = mSigmaPoints.unscentedTransform(measurementPoints);
            measurementDistribution.covariance.template triangularView<Eigen::Lower>() += measurement.covariance;
            measurementDistribution.covariance.template triangularView<Eigen::Upper>() = measurementDistribution.covariance.transpose();

            Eigen::Matrix<T, N, S> gain;
            gain.setZero();

            for (int i = 0; i < mSigmaPoints.size(); ++i) {
                auto weight = mSigmaPoints.covarianceWeight(i);
                auto stateDifference = points[i] - mState.mean;
                auto measurementDifference = measurementPoints[i] - measurementDistribution.mean;

                gain += weight * stateDifference * measurementDifference.transpose();
            }

            gain *= measurementDistribution.covariance.inverse();

            mState.mean += gain * (measurement.mean - measurementDistribution.mean);
            mState.covariance.template triangularView<Eigen::Lower>() -= gain * measurementDistribution.covariance * gain.transpose();
        }

        void reset()
        {
            mState.mean.setZero();
            mState.covariance.setZero();
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
