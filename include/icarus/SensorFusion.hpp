#pragma once

#include "State.hpp"

#include <cstddef>

namespace icarus
{
    struct SensorReading;

    struct SensorFusion
    {
        virtual ~SensorFusion() = default;
        virtual State state() const = 0;
    };
}
