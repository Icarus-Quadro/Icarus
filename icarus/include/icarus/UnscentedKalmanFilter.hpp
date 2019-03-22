#pragma once

#include "MerweScaledSigmaPoints.hpp"
#include "GaussianDistribution.hpp"

#include <iostream>
#include <Eigen/Dense>

namespace icarus
{
    template<typename T, size_t N, size_t M>
    GaussianDistribution<T, N> unscentedTransform(SigmaPoints<T, N, M> const & points)
    {
        GaussianDistribution<T, N> ret;
        ret.mean.setZero();
        ret.covariance.setZero();

        for (int i = 0; i < M; ++i) {
            ret.mean += points.meanWeights[i] * points.points[i];
        }

        for (int i = 0; i < M; ++i) {
            auto difference = (points.points[i] - ret.mean).eval();
            ret.covariance += points.covarianceWeights[i] * difference * difference.transpose();
        }

        return ret;
    }

    template<typename T, size_t N, typename P, typename M, size_t S>
    GaussianDistribution<T, N> UnscentedKalmanFilter(GaussianDistribution<T, N> const & state, P const & processModel, M const & measurementModel, GaussianDistribution<T, S> const & measurement)
    {
        // predict
        auto sigmaPoints = MerweScaledSigmaPoints(state, T(0.1));

        for (auto & point : sigmaPoints.points) {
            point = processModel(point, T(0.01)); // TODO don't hardcode time step
        }

        auto newState = unscentedTransform(sigmaPoints);
        newState.covariance += processModel.noise();

        //update
        SigmaPoints<T, S, sigmaPoints.size()> measurementPoints;
        measurementPoints.meanWeights = sigmaPoints.meanWeights;
        measurementPoints.covarianceWeights = sigmaPoints.covarianceWeights;

        for (int i = 0; i < sigmaPoints.size(); ++i) {
            measurementPoints.points[i] = measurementModel(sigmaPoints.points[i]);
        }

        auto measurementDistribution = unscentedTransform(measurementPoints);
        measurementDistribution.covariance += measurement.covariance;

        Eigen::Matrix<T, N, S> gain;
        gain.setZero();

        for (int i = 0; i < sigmaPoints.size(); ++i) {
            auto weight = sigmaPoints.covarianceWeights[i];
            auto stateDifference = sigmaPoints.points[i] - newState.mean;
            auto measurementDifference = measurementPoints.points[i] - measurementDistribution.mean;

            gain += weight * stateDifference * measurementDifference.transpose();
        }

        gain *= measurementDistribution.covariance.inverse();

        auto residual = (measurement.mean - measurementDistribution.mean).eval();
        newState.mean += gain * residual;
        newState.covariance -= gain * measurementDistribution.covariance * gain.transpose();

        return newState;
    }
}
