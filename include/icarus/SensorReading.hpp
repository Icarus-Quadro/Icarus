#pragma once

#include "Algebra.hpp"

namespace icarus
{
    enum class SensorReadingType
    {
        position,
        acceleration,
        angularVelocity,
        magneticField,
    };

    struct SensorReading
    {
        SensorReadingType type;
        union {
            types::Vector3 position;
            types::Vector3 acceleration;
            types::Vector3 angularVelocity;
            types::Vector3 magneticField;
        };
    };
}
