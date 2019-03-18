#include <iostream>


#include <icarus/UnscentedKalmanFilter.hpp>
#include <icarus/GyroscopeMeasurementModel.hpp>
#include <icarus/RigidBodyProcessModel.hpp>

#include <gtest/gtest.h>

TEST(KalmanFilter, ShouldReturnNormalizedOrientation)
{
    icarus::GaussianDistribution<float, 7> state;
    state.mean << 0, 0, 0, 1, 0, 0, 0;
    state.covariance.setIdentity();
    state.covariance *= 0.01f;

    auto & s = reinterpret_cast<icarus::RigidBodyProcessModel<float>::State &>(state.mean);

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    for (int i = 0; i < 10; ++i) {
        icarus::GaussianDistribution<float, 3> measurement;
        measurement.mean << 0.0f, 0.0f, 0.0f;
        measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;

        state = UnscentedKalmanFilter(state, processModel, measurementModel, measurement);

        auto norm = s.orientation.norm();
        ASSERT_NEAR(norm, 1.0f, 0.01f);
    }
}
