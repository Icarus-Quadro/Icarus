#include <icarus/UnscentedKalmanFilter.hpp>
#include <icarus/GyroscopeMeasurementModel.hpp>
#include <icarus/RigidBodyProcessModel.hpp>

#include <gtest/gtest.h>

TEST(UnscentedTransform, IsLinearForLinearProblems)
{
    icarus::GaussianDistribution<float, 3> expected;
    expected.mean << 1, 2, 3;
    expected.covariance.setIdentity();

    auto sigmaPoints = icarus::MerweScaledSigmaPoints(expected, 0.1f);
    auto actual = icarus::unscentedTransform(sigmaPoints);

    EXPECT_NEAR((expected.mean - actual.mean).norm(), 0.0f, 0.0001f);
    EXPECT_NEAR((expected.covariance - actual.covariance).norm(), 0.0f, 0.0001f);
}

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
    } processModel;

    struct {
        Eigen::Matrix<float, 1, 1> operator()(Eigen::Matrix<float, 2, 1> const & state) const
        {
            Eigen::Matrix<float, 1, 1> ret;
            ret << state[1];
            return ret;
        }
    } measurementModel;

    icarus::GaussianDistribution<float, 2> state;
    state.mean << 0, 0;
    state.covariance.setIdentity();
    state.covariance *= 0.01f;

    for (int i = 0; i < 100; ++i) {
        icarus::GaussianDistribution<float, 1> measurement;
        measurement.mean << 1.0f;
        measurement.covariance << 0.001f;

        state = UnscentedKalmanFilter(state, processModel, measurementModel, measurement);

        ASSERT_NEAR(state.mean[0], (i + 1) / 100.0f, 0.05f);
        ASSERT_NEAR(state.mean[1], 1.0f, 0.2f);
    }
}

TEST(KalmanFilter, Rotates180DegreesAroundX)
{
    icarus::GaussianDistribution<float, 7> state;
    state.mean << 0, 0, 0, 0, 0, 0, 1;
    state.covariance.setIdentity();
    state.covariance *= 0.1f;

    auto & s = reinterpret_cast<icarus::RigidBodyProcessModel<float>::State &>(state.mean);

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    for (int i = 0; i < 50; ++i) {
        icarus::GaussianDistribution<float, 3> measurement;
        measurement.mean << 3.1415f, 0.0f, 0.0f;
        measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.00001f;
        state = UnscentedKalmanFilter(state, processModel, measurementModel, measurement);
        s.orientation.normalize();
    }

    auto actual = s.orientation;
    auto expected = Eigen::Quaternionf(std::sqrt(0.5f), std::sqrt(0.5f), 0, 0);
    EXPECT_NEAR(std::abs(expected.dot(actual)), 1.0f, 0.01f);
}
