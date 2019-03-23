#include <icarus/UnscentedKalmanFilter.hpp>
#include <icarus/GyroscopeMeasurementModel.hpp>
#include <icarus/RigidBodyProcessModel.hpp>

#include <gtest/gtest.h>

#include "IsSameOrientationAs.hpp"


TEST(KalmanFilter, WorksForLinearProblems)
{
    // state = [position, velocity]

    struct {
        Eigen::Matrix<float, 2, 1> operator()(Eigen::Matrix<float, 2, 1> const & state, float timeStep) const
        {
            Eigen::Matrix<float, 2, 1> ret;
            ret << state[0] + state[1] * timeStep, state[1];
            return ret;
        }
        Eigen::Matrix<float, 2, 2> noise() const
        {
            Eigen::Matrix<float, 2, 2> ret;
            ret.setIdentity();
            ret *= 0.001f;
            return ret;
        }
    } processModel;

    struct {
        Eigen::Matrix<float, 1, 1> operator()(Eigen::Matrix<float, 2, 1> const & state) const
        {
            Eigen::Matrix<float, 1, 1> ret;
            ret << state[1];
            return ret;
        }
    } measurementModel;

    icarus::UnscentedKalmanFilter<float, 2> kalman;

    icarus::GaussianDistribution<float, 1> measurement;
    measurement.mean << 1.0f;
    measurement.covariance << 0.001f;
    for (int i = 0; i < 100; ++i) {
        kalman.filter(processModel, measurementModel, measurement, 0.01f);

        auto & state = kalman.stateVector();
        ASSERT_NEAR(state[0], (i + 1) / 100.0f, 0.05f);
        ASSERT_NEAR(state[1], 1.0f, 0.2f);
    }
}

TEST(KalmanFilter, Rotates90DegreesAroundX)
{
    using State = icarus::RigidBodyProcessModel<float>::State;

    icarus::UnscentedKalmanFilter<float, 7> kalman;
    auto & s = kalman.state<State>();
    s.orientation.setIdentity();

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    for (int i = 0; i < 50; ++i) {
        icarus::GaussianDistribution<float, 3> measurement;
        measurement.mean << 3.1415f, 0.0f, 0.0f;
        measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        s.orientation.normalize();
    }

    auto actual = s.orientation;
    Eigen::Quaternionf expected;
    expected = Eigen::AngleAxisf(0.5 * M_PI,  Eigen::Vector3f::UnitX());
    EXPECT_THAT(actual, IsSameOrientationAs(expected, 0.05f));
}

TEST(KalmanFilter, Around3Axes)
{
    using State = icarus::RigidBodyProcessModel<float>::State;

    icarus::UnscentedKalmanFilter<float, 7> kalman;
    auto & s = kalman.state<State>();
    s.orientation.setIdentity();
    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;
    icarus::GaussianDistribution<float, 3> measurement;

    // 90 around x
    measurement.mean << 3.1415f, 0.0f, 0.0f;
    measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;
    for (int i = 0; i < 50; ++i) {
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        s.orientation.normalize();
    }

    // 90 around y
    measurement.mean << 0, 3.1415f, 0.0f;
    measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;
    for (int i = 0; i < 50; ++i) {
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        s.orientation.normalize();
    }

    // 90 around z
    measurement.mean << 0, 0, 3.1415f;
    measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;
    for (int i = 0; i < 50; ++i) {
        kalman.filter(processModel, measurementModel, measurement, 0.01f);
        s.orientation.normalize();
    }

    auto actual = s.orientation;

    using Vec = Eigen::Vector3f;
    using Quat = Eigen::Quaternionf;
    using AA = Eigen::AngleAxisf;

    Quat expected;
    expected = AA(0.5 * M_PI,  Vec::UnitX()) * AA(0.5 * M_PI,  Vec::UnitY()) * AA(0.5 * M_PI,  Vec::UnitZ());
    EXPECT_THAT(actual, IsSameOrientationAs(expected, 0.05f));
}
