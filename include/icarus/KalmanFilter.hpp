#pragma once

#include "SensorFusion.hpp"

#include <vector>

namespace icarus
{
    struct SensorReading;

    struct KalmanFilter : SensorFusion
    {
        explicit KalmanFilter(std::initializer_list<Sensor *> sensors);
        virtual State state() const = 0;
    private:
        std::vector<Sensor *> mSensors;
    };
}
