#include <icarus/sensor/EllipsoidalCalibrator.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>

using namespace testing;

MATCHER(VectorsEq, "") {
    return get<0>(arg).isApprox(get<1>(arg), 0.01);
}

TEST(EllipsoidalCalibrator, Works)
{
    std::vector<Eigen::Matrix<float, 3, 1>> samples{
        {0.22688f, 0.63440f, -0.02708f, },
        {-0.03061f, 0.86715f, 0.25525f, },
        {0.14552f, 0.78912f, 0.02933f, },
        {0.01972f, 0.71303f, 0.19761f, },
        {0.00524f, 0.77114f, 0.18609f, },
        {-0.02602f, 0.86270f, 0.11841f, },
        {0.09043f, 0.71599f, 0.10080f, },
        {0.01620f, 0.74215f, 0.28993f, },
        {-0.05429f, 1.00592f, 0.13624f, },
        {0.15229f, 0.68029f, 0.01052f, },
        {0.13919f, 0.62491f, 0.08971f, },
        {-0.04478f, 0.78341f, 0.22090f, },
        {0.26740f, 0.50252f, -0.01775f, },
        {0.27435f, 0.83616f, 0.06213f, },
        {0.23777f, 0.56827f, 0.01963f, },
        {0.04849f, 0.63102f, 0.32882f, },
        {-0.00899f, 0.93560f, 0.15985f, },
        {0.12387f, 0.98701f, 0.21342f, },
        {0.07250f, 0.85025f, -0.03232f, },
        {0.12676f, 0.85293f, 0.21056f, },
    };

    std::vector<Eigen::Matrix<float, 3, 1>> expected
    {
        {0.24769f, -0.40299f, -0.95915f, },
        {-0.83149f, 0.32192f, 0.50158f, },
        {-0.01590f, 0.19511f, -0.69252f, },
        {-0.74883f, -0.30966f, 0.21127f, },
        {-0.77152f, -0.06341f, 0.13251f, },
        {-0.88307f, 0.31890f, -0.27995f, },
        {-0.38460f, -0.20535f, -0.30606f, },
        {-0.68597f, -0.19029f, 0.73606f, },
        {-0.87102f, 0.93046f, -0.20513f, },
        {-0.11372f, -0.28640f, -0.78743f, },
        {-0.21255f, -0.55832f, -0.33577f, },
        {-1.03041f, -0.06954f, 0.30429f, },
        {0.33489f, -0.95142f, -0.87272f, },
        {0.80820f, 0.55513f, -0.43971f, },
        {0.25889f, -0.69262f, -0.67997f, },
        {-0.60677f, -0.65725f, 0.98572f, },
        {-0.67630f, 0.66415f, -0.03953f, },
        {0.18748f, 1.04684f, 0.33410f, },
        {-0.40343f, 0.39078f, -1.09017f, },
        {0.04607f, 0.44551f, 0.33074f, },
    };

    icarus::EllipsoidalCalibrator<float> magCal(samples.size());

    for (auto & s : samples) {
        magCal.addSample(s);
    }

    auto cal = magCal.computeCalibration(1.0f);

    std::vector<Eigen::Matrix<float, 3, 1>> actual;

    for (auto & s : samples) {
        auto a = cal.adjust(s);
        actual.push_back(a);
    }

    EXPECT_THAT(actual, Pointwise(VectorsEq(), expected));
}