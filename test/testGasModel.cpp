#include <icarus/sensor/GasModel.hpp>

#include <gtest/gtest.h>

TEST(GasModel, Works)
{
    icarus::GasModel<float> gm;

    float h0 = gm.altitude(101325);
    ASSERT_NEAR(h0, 0.0f, 0.00001f);

    float h1000 = gm.altitude()
}
