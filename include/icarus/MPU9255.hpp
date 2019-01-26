#pragma once

#include "SensorReading.hpp"

namespace icarus
{
    template<typename RegisterBank>
    struct MPU9255
    {
        explicit MPU9255(RegisterBank * device);
        void initialize();
        void read();

        SensorReading acceleration() const
        {
            SensorReading ret;
            ret.type = SensorReadingType::acceleration;
            ret.acceleration = mAcceleration;
            return ret;
        }

        SensorReading angularVelocity() const
        {
            SensorReading ret;
            ret.type = SensorReadingType::angularVelocity;
            ret.angularVelocity = mAngularVelocity;
            return ret;
        }
    private:
        RegisterBank * mDevice;

        types::Scalar mAccelerationScale;
        types::Scalar mAngluarVelocityScale;

        types::Scalar mTemperature;
        types::Vector3 mAcceleration;
        types::Vector3 mAngularVelocity;
    };
}