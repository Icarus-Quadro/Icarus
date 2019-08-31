#include "TestBase.hpp"

#include <icarus/sensorFusion/GasModel.hpp>

TEST(GasModel, Works)
{
    icarus::GasModel<float> gm;

    float h0 = gm.altitude(101325);
    EXPECT_NEAR(h0, 0.0f, 0.00001f);

    // float h1000 = gm.altitude(1000.0f);
    // EXPECT_NEAR(h0, 0.0f, 0.00001f);
}
