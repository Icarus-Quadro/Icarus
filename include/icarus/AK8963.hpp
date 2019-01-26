#pragma once

#include "SensorReading.hpp"

namespace icarus
{
    template<typename RegisterBank>
    struct AK8963
    {
        explicit AK8963(RegisterBank * device);
        void initialize();
        void read();

        SensorReading magneticField() const
        {
            SensorReading ret;
            ret.type = SensorReadingType::magneticField;
            ret.magneticField = mMagneticField;
            return ret;
        }
    private:
        RegisterBank * mDevice;

        types::Vector3 mScale;
        types::Vector3 mMagneticField;
    };
}