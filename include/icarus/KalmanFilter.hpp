#pragma once

#include "MerweScaledSigmaPoints.hpp"

#include <Eigen/Dense>

namespace icarus
{
    template<typename T, typename N>
    struct KalmanFilter
    {
        explicit KalmanFilter();
        State state() const;
        void integrateReadings(float timeStep, Eigen::Matrix<float, 3, 1> const & angularVelocity, Eigen::Matrix<float, 3, 1> const & magneticField);

        template<typename P, typename M>
        void predict(P const & processModel, M const & measurementModel);
    private:

        Eigen::Matrix<T, N, 1> mMean;
        Eigen::Matrix<T, N, N> mCovariance;
    };

    template<typename T, typename N>
    template<typename P, typename M, size_t S>
    void KalmanFilter<T, N>::predict(P const & processModel, M const & measurementModel, Eigen::Matrix<T, S, 1> const & measurement)
    {
        // predict
        auto sigmaPoints = MerweScaledSigmaPoints(mMean, mCovariance, 0.5, 2, 3 - N);

        mMean.setZero();
        mCovariance.setZero();
        for (auto & point : sigmaPoints) {
            point.point = processModel(point.point);
            mMean += point.meanWeight * point.point;
        }

        for (auto & point : sigmaPoints) {
            auto difference = (point.point - mMean).eval();
            mCovariance += point.covarianceWeight * difference * difference.transpose();
        }

        // mCovariance += processModel.noise();

        //update
        using Measurement = Eigen::Matrix<T, S, 1>;
        std::array<Measurement, sigmaPoints.size()> measurements;

        Measurement measurementMean;
        measurementMean.setZero();
        for (int i = 0; i < sigmaPoints.size(); ++i) {
            auto & sigmaPoint = sigmaPoints[i];
            measurements[i] = measurementModel(sigmaPoint.point);
            mMean += sigmaPoint.meanWeight * measurements[i];
        }

        Eigen::Matrix<T, S, S> measurementCovariance;
        measurementCovariance.setZero();

        for (int i = 0; i < sigmaPoints.size(); ++i) {
            auto & point = measurements[i];
            auto weight = sigmaPoints[i].covarianceWeight;
            auto difference = (point - measurementMean).eval();
            measurementCovariance += weight * difference * difference.transpose();
        }

        measurementCovariance += measurementModel.noise();

        Eigen::Matrix<T, S, N> gain;
        gain.setZero();

        for (int i = 0; i < sigmaPoints.size(); ++i) {
            auto & statePoint = sigmaPoints[i];
            auto & measurementPoint = measurements[i];
            auto weight = statePoint.covarianceWeight;
            measurementCovariance += weight * (statePoint.point - mMean) * (measurementPoint - measurementMean).transpose();
        }

        gain *= measurementCovariance.inverse();

        Eigen::Matrix<T, S, 1> inovation = measurement - measurementMean;

        mMean += gain * innovation;
        mCovariance -= gain * measurementCovariance * gain.transpose();
    }
}
