#include "TestBase.hpp"

#include <icarus/sensorFusion/UnscentedKalmanFilter.hpp>
#include <icarus/sensorFusion/FlightModel.hpp>

using Model = icarus::FlightModel<float>;

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
    measurement.covariance << 0.0001f;
    for (int i = 0; i < 100; ++i) {
        kalman.filter(processModel, measurementModel, measurement, 0.01f);

        auto & state = kalman.stateVector();
        ASSERT_NEAR(state[0], (i + 1) / 100.0f, 0.05f);
        ASSERT_NEAR(state[1], 1.0f, 0.2f);
    }
}

struct KalmanTest : testing::Test
{
    KalmanTest() :
        s(kalman.stateVector()),
        measurement(measurementDistribution.mean)
    {
        kalman.reset();
        s.reset();
        measurementDistribution.mean.setZero();
        measurement.pressure() = 101325.0f;
        measurementDistribution.covariance.setZero();
        measurementDistribution.covariance.diagonal().setConstant(0.0001f);

        iterate(10, 0.01f);
    }

    void iterate(size_t count, float timeStep)
    {
        for (size_t i = 0; i < count; ++i) {
            kalman.filter(processModel, measurementModel, measurementDistribution, timeStep);
            s.orientation().normalize();
        }
    }

    using Model = icarus::FlightModel<float>;
    using Vec = Eigen::Vector3f;
    using Quat = Eigen::Quaternionf;
    using AA = Eigen::AngleAxisf;

    icarus::UnscentedKalmanFilter<float, Model::StateSize> kalman;
    Model::State s;
    Model::ProcessModel processModel;
    Model::MeasurementModel measurementModel;
    Model::MeasurementDistribution measurementDistribution;
    Model::Measurement measurement;
};

TEST_F(KalmanTest, Rotates90DegreesAroundX)
{
    measurement.angularVelocity() << M_PI, 0.0f, 0.0f;

    iterate(50, 0.01f);

    Quat expected;
    expected = AA(0.5 * M_PI,  Eigen::Vector3f::UnitX());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, Rotates90DegreesAroundY)
{
    measurement.angularVelocity() << 0.0f, M_PI, 0.0f;

    iterate(50, 0.01f);

    Quat expected;
    expected = AA(0.5 * M_PI,  Eigen::Vector3f::UnitY());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, Rotates90DegreesAroundZ)
{
    measurement.angularVelocity() << 0.0f, 0.0f, M_PI;
    iterate(50, 0.01f);

    Quat expected;
    expected = AA(0.5 * M_PI,  Eigen::Vector3f::UnitZ());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, Rotates360DegreesAroundX)
{
    measurement.angularVelocity() << M_PI, 0.0f, 0.0f;
    iterate(200, 0.01f);

    Quat expected;
    expected = AA(2 * M_PI,  Eigen::Vector3f::UnitX());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, RotatesThereAndBack)
{
    measurement.angularVelocity() << 0.0f, 0.0f, M_PI;
    iterate(50, 0.01f);
    measurement.angularVelocity() << 0.0f, 0.0f, -M_PI;
    iterate(50, 0.01f);

    Quat expected;
    expected.setIdentity();
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, Around2Axes)
{
    // 90 around y
    measurement.angularVelocity() << 0, M_PI, 0.0f;
    iterate(50, 0.01f);

    // 90 around x
    measurement.angularVelocity() << M_PI, 0.0f, 0.0f;
    iterate(50, 0.01f);

    Quat expected;
    expected = AA(0.5 * M_PI,  Vec::UnitY()) * AA(0.5 * M_PI,  Vec::UnitX());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}

TEST_F(KalmanTest, Around3Axes)
{
    // 90 around y
    measurement.angularVelocity() << 0, M_PI, 0.0f;
    iterate(50, 0.01f);

    // 90 around x
    measurement.angularVelocity() << M_PI, 0.0f, 0.0f;
    iterate(50, 0.01f);

    // 90 around z
    measurement.angularVelocity() << 0, 0, M_PI;
    iterate(50, 0.01f);

    Quat expected;
    expected = 
        AA(0.5 * M_PI,  Vec::UnitY()) * 
        AA(0.5 * M_PI,  Vec::UnitX()) * 
        AA(0.5 * M_PI,  Vec::UnitZ());
    EXPECT_THAT(s.orientation(), IsSameOrientation(expected, 0.05f));
}
