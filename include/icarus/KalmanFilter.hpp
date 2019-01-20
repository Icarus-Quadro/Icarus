#pragma once

#include "SensorFusion.hpp"
#include "SensorReading.hpp"

namespace icarus
{
    struct SensorReading;

    struct KalmanFilter : SensorFusion
    {
        explicit KalmanFilter();
        State state() const override;
        void integrateReadings(SensorReading const * readings, size_t size, types::Scalar timeDelta);
    };
}
