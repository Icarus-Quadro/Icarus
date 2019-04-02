#pragma once

#include <Eigen/Dense>

namespace icarus
{
    template<typename RegisterBank>
    struct MPU9255
    {
        explicit MPU9255(RegisterBank * device);
        void initialize();
        void i2cBypass(bool enable);
        void read();

        float temperature() const
        {
            return mTemperature;
        }

        Eigen::Matrix<float, 3, 1> acceleration() const
        {
            return mAcceleration;
        }

        Eigen::Matrix<float, 3, 1> angularVelocity() const
        {
            return mAngularVelocity;
        }
    private:
        RegisterBank * mDevice;

        float mAccelerationScale;
        int mGyroscopeScaleRange;

        float mTemperature;
        Eigen::Matrix<float, 3, 1> mAcceleration;
        Eigen::Matrix<float, 3, 1> mAngularVelocity;
    };
}