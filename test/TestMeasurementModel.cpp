#include "TestBase.hpp"

#include <icarus/sensorFusion/FlightModel.hpp>

using Model = icarus::FlightModel<float>;
using namespace Eigen;

struct MeasurementModelTest : testing::Test
{
    explicit MeasurementModelTest() :
        state(mState),
        actual(mActualMeasurement)
    {
        state.reset();
    }

    void process()
    {
        mActualMeasurement = model(mState);
    }

private:
    Model::StateVector mState;
    Model::MeasurementVector mActualMeasurement;
public:
    Model::State state;
    Model::Measurement actual;
    Model::MeasurementModel model;
};

constexpr float epsilon = 0.0001f;

TEST_F(MeasurementModelTest, ExperiencesGravity)
{
    process();
    EXPECT_THAT(actual.acceleration(), IsSameMatrix(Vector3f(0, 0, 9.81), epsilon));

    state.orientation() = AngleAxisf(M_PI / 2, Vector3f::UnitZ());
    process();
    EXPECT_THAT(actual.acceleration(), IsSameMatrix(Vector3f(0, 0, 9.81), epsilon));

    state.orientation() = AngleAxisf(M_PI / 2, Vector3f::UnitX());
    process();
    EXPECT_THAT(actual.acceleration(), IsSameMatrix(Vector3f(0, 9.81, 0), epsilon));

    state.orientation() = AngleAxisf(M_PI / 2, Vector3f::UnitY());
    process();
    EXPECT_THAT(actual.acceleration(), IsSameMatrix(Vector3f(-9.81, 0, 0), epsilon));
}

TEST_F(MeasurementModelTest, MeasuresAngularVelocity)
{
    Vector3f angularVelocity(1, 2, 3);
    state.angularMomentum() << angularVelocity;
    process();
    EXPECT_THAT(actual.angularVelocity(), IsSameMatrix(angularVelocity, epsilon));
}

TEST_F(MeasurementModelTest, MeasuresAngularVelocityInRotatedFrame)
{
    state.angularMomentum() << 1, 0, 0;
    state.orientation() = AngleAxisf(M_PI / 2, Vector3f::UnitZ());
    process();
    EXPECT_THAT(actual.angularVelocity(), IsSameMatrix(Vector3f(0, -1, 0), epsilon));
}

TEST_F(MeasurementModelTest, DoesNotMeasurePositionAndVelocity)
{
    Vector3f magneticField(0.7, 0.7, 0);
    model.referenceMagneticField(magneticField);
    state.position() << 1, 2, 3;
    state.velocity() << -32, 43, 67;
    process();

    EXPECT_THAT(actual.angularVelocity(), IsSameMatrix(Vector3f(0, 0, 0), epsilon));
    EXPECT_THAT(actual.magneticField(), IsSameMatrix(magneticField, epsilon));
    EXPECT_THAT(actual.acceleration(), IsSameMatrix(Vector3f(0, 0, 9.81), epsilon));
}

TEST_F(MeasurementModelTest, MeasuresPressure)
{
    process();
    auto const refPressure = actual.pressure();

    state.position() << 233, 43, 0;
    process();
    EXPECT_EQ(actual.pressure(), refPressure);

    state.position() << 0, 0, 100;
    process();
    EXPECT_LT(actual.pressure(), refPressure);

    state.position() << 0, 0, -100;
    process();
    EXPECT_GT(actual.pressure(), refPressure);
}

