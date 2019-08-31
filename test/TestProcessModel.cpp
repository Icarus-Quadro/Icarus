#include "TestBase.hpp"

#include <icarus/sensorFusion/FlightModel.hpp>

using Model = icarus::FlightModel<float>;
using namespace Eigen;

struct ProcessModelTest : testing::Test
{
    explicit ProcessModelTest() :
        state(mState)
    {
        state.reset();
    }

    void process(float timeStep)
    {
        mState = model(mState, timeStep);
    }

private:
    Model::ProcessModel model;
    Model::StateVector mState;
public:
    Model::State state;
};

constexpr float epsilon = 0.0001f;

TEST_F(ProcessModelTest, DoesNotChangeWhenStationary)
{
    process(1.0f);
    EXPECT_THAT(state.position(), IsSameMatrix(Vector3f::Zero(), epsilon));
    EXPECT_THAT(state.velocity(), IsSameMatrix(Vector3f::Zero(), epsilon));
    EXPECT_THAT(state.acceleration(), IsSameMatrix(Vector3f::Zero(), epsilon));
    EXPECT_THAT(state.orientation(), IsSameOrientation(Quaternionf::Identity(), epsilon));
    EXPECT_THAT(state.angularMomentum(), IsSameMatrix(Vector3f::Zero(), epsilon));
}

TEST_F(ProcessModelTest, AngularMomentumIsPreserved)
{
    Vector3f value(1, 0, 0);   
    state.angularMomentum() = value;
    process(0.1f);
    
    EXPECT_THAT(state.angularMomentum(), IsSameMatrix(value, epsilon));
}

TEST_F(ProcessModelTest, Rotates90DegreesAroundXAxis)
{
    state.angularMomentum() << M_PI, 0, 0;
    process(1.0f);
    Quaternionf expected;
    expected = AngleAxisf(M_PI, Vector3f::UnitX());
    EXPECT_THAT(state.orientation(), IsSameOrientation(expected, epsilon));
}

TEST_F(ProcessModelTest, Rotates90DegreesAroundYAxis)
{
    state.angularMomentum() << 0, M_PI, 0;
    process(1.0f);
    Quaternionf expected;
    expected = AngleAxisf(M_PI, Vector3f::UnitY());
    EXPECT_THAT(state.orientation(), IsSameOrientation(expected, epsilon));
}

TEST_F(ProcessModelTest, Rotates90DegreesAroundZAxis)
{
    state.angularMomentum() << 0, 0, M_PI;
    process(1.0f);
    Quaternionf expected;
    expected = AngleAxisf(M_PI, Vector3f::UnitZ());
    EXPECT_THAT(state.orientation(), IsSameOrientation(expected, epsilon));
}

TEST_F(ProcessModelTest, RotatesDegreesAround3Axes)
{
    state.angularMomentum() << M_PI, 0, 0;
    process(1.0f);
    state.angularMomentum() << 0, M_PI, 0;
    process(1.0f);
    state.angularMomentum() << 0, 0, M_PI;
    process(1.0f);

    Quaternionf expected;
    expected =
        AngleAxisf(M_PI, Vector3f::UnitX()) *
        AngleAxisf(M_PI, Vector3f::UnitY()) *
        AngleAxisf(M_PI, Vector3f::UnitZ());
    EXPECT_THAT(state.orientation(), IsSameOrientation(expected, epsilon));
}

TEST_F(ProcessModelTest, RotatesDegreesAround2AxesSimultaneously)
{
    state.angularMomentum() << sqrt(0.5), sqrt(0.5), 0;
    process(M_PI);

    Quaternionf expected;
    expected =
        AngleAxisf(-M_PI / 2, Vector3f::UnitZ()) *
        AngleAxisf(M_PI, Vector3f::UnitY());
    EXPECT_THAT(state.orientation(), IsSameOrientation(expected, epsilon));
}

TEST_F(ProcessModelTest, MovesAlongStraightLineWithInitialVelocity)
{
    state.velocity() << 1, 2, 3;
    
    process(1.0f);
    EXPECT_THAT(state.position(), IsSameMatrix(Vector3f(1, 2, 3), epsilon));
    process(1.0f);
    EXPECT_THAT(state.position(), IsSameMatrix(Vector3f(2, 4, 6), epsilon));
}

TEST_F(ProcessModelTest, Accelerates)
{
    state.acceleration() << 1, 0, 0;
    
    process(1.0f);
    EXPECT_THAT(state.position(), IsSameMatrix(Vector3f(0.5, 0, 0), epsilon));
    EXPECT_THAT(state.velocity(), IsSameMatrix(Vector3f(1, 0, 0), epsilon));

    process(1.0f);
    EXPECT_THAT(state.position(), IsSameMatrix(Vector3f(2, 0, 0), epsilon));
    EXPECT_THAT(state.velocity(), IsSameMatrix(Vector3f(2, 0, 0), epsilon));
}

