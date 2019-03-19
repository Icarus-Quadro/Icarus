#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <icarus/GaussianDistribution.hpp>
#include <icarus/MerweScaledSigmaPoints.hpp>

#include <numeric>

using namespace testing;

TEST(MerweScaledSigmaPoints, 2DWeights)
{
    icarus::GaussianDistribution<float, 2> dist;
    dist.mean << 0, 0;
    dist.covariance << 1, 0,
                       0, 1;

    auto sigmaPoints = icarus::MerweScaledSigmaPoints(dist, 0.5f);

    std::array<float, 5> expectedMeanWeights{-1.6666666f, 0.6666666f, 0.6666666f, 0.6666666f, 0.6666666f};
    std::array<float, 5> expectedCovarianceWeights{1.08333333f, 0.6666666f, 0.6666666f, 0.6666666f, 0.6666666f};

    EXPECT_THAT(sigmaPoints.meanWeights, Pointwise(FloatEq(), expectedMeanWeights));
    EXPECT_THAT(sigmaPoints.covarianceWeights, Pointwise(FloatEq(), expectedCovarianceWeights));
}

TEST(MerweScaledSigmaPoints, 10MeanWeightsSumToOne)
{
    icarus::GaussianDistribution<float, 10> dist;
    dist.mean << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

    Eigen::Matrix<float, 10, 10> covar;
    covar.setIdentity();
    covar(1, 5) = 23;
    covar(6, 7) = 1;
    covar(8, 5) = -4;

    dist.covariance = covar * covar.transpose();

    auto sigmaPoints = icarus::MerweScaledSigmaPoints(dist, 0.5f);
    auto weightSum = std::accumulate(sigmaPoints.meanWeights.begin(), sigmaPoints.meanWeights.end(), 0.0f);
    EXPECT_NEAR(weightSum, 1.0f, 0.00001f);
}

TEST(MerweScaledSigmaPoints, 2DValues)
{
    icarus::GaussianDistribution<float, 2> dist;
    dist.mean << 1, -1;
    dist.covariance << 2, -1,
                       -1, 4;

    auto sigmaPoints = icarus::MerweScaledSigmaPoints(dist, 0.5f);

    std::array<Eigen::Vector2f, 5> expectedValues{
        Eigen::Vector2f( 1.0f,        -1.0f),
        Eigen::Vector2f( 2.22474487f, -1.61237244f ),
        Eigen::Vector2f(-0.22474487,  -0.38762756f ),
        Eigen::Vector2f( 1.0f,         0.62018517f ),
        Eigen::Vector2f( 1.0f,        -2.62018517f ),
    };

    for (int i = 0; i < 5; ++i) {
        auto expected = expectedValues[i];
        auto actual = sigmaPoints.points[i];

        for (int j = 0; j < 2; ++j) {
            EXPECT_FLOAT_EQ(expected[j], actual[j]);
        }
    }
}
