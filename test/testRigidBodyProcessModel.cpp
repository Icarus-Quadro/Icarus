#include <icarus/RigidBodyProcessModel.hpp>

#include <gtest/gtest.h>

using Model = icarus::RigidBodyProcessModel<float>;

void expectStatesToBeEqual(Eigen::Matrix<float, 7, 1> const & expectedVec, Eigen::Matrix<float, 7, 1> const & actualVec)
{
    auto expected = reinterpret_cast<Model::State const &>(expectedVec);
    auto actual = reinterpret_cast<Model::State const &>(actualVec);

    EXPECT_NEAR(std::abs(expected.orientation.dot(actual.orientation)), 1.0f, 0.001f);
    EXPECT_FLOAT_EQ((expected.angularMomentum - actual.angularMomentum).norm(), 0.0f);
}

TEST(RigidBodyProcessModel, Rotates360Degrees)
{
    Model processModel;

    Eigen::Matrix<float, 7, 1> initState, state;
    // initial state
    // angular velocity 2 * pi rad/s
    // uint orientation
    initState << 2 * 3.1415f, 0, 0, 0, 0, 0, 1;
    state = processModel(initState, 1.0f);
    expectStatesToBeEqual(initState, state);

    initState << 0, 2 * 3.1415f, 0, 0, 0, 0, 1;
    state = processModel(initState, 1.0f);
    expectStatesToBeEqual(initState, state);

    initState << 0, 0, 2 * 3.1415f, 0, 0, 0, 1;
    state = processModel(initState, 1.0f);
    expectStatesToBeEqual(initState, state);
}

constexpr float quaterTurn = 0.5f * 3.141592f;
constexpr float K = std::sqrt(0.5f);


TEST(RigidBodyProcessModel, Rotates90DegreesAroundXAxis)
{
    Model processModel;
    Eigen::Matrix<float, 7, 1> state, expected;
    state << quaterTurn, 0, 0, 0, 0, 0, 1;
    expected <<  quaterTurn, 0, 0, K, 0, 0, K;
    state = processModel(state, 1.0f);
    expectStatesToBeEqual(expected, state);
}

TEST(RigidBodyProcessModel, Rotates90DegreesAroundYAxis)
{
    Model processModel;
    Eigen::Matrix<float, 7, 1> state, expected;
    state << 0, quaterTurn, 0, 0, 0, 0, 1;
    expected <<  0, quaterTurn, 0, 0, K, 0, K;
    state = processModel(state, 1.0f);
    expectStatesToBeEqual(expected, state);
}

TEST(RigidBodyProcessModel, Rotates90DegreesAroundZAxis)
{
    Model processModel;
    Eigen::Matrix<float, 7, 1> state, expected;
    state << 0, 0, quaterTurn, 0, 0, 0, 1;
    expected <<  0, 0, quaterTurn, 0, 0, K, K;
    state = processModel(state, 1.0f);
    expectStatesToBeEqual(expected, state);
}