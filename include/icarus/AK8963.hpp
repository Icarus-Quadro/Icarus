#pragma once

#include <Eigen/Dense>

namespace icarus
{
    template<typename Delay, typename RegisterBank>
    struct AK8963
    {
        explicit AK8963(RegisterBank * device);
        void initialize();
        void read();

        Eigen::Matrix<float, 3, 1> magneticField() const
        {
            return mMagneticField;
        }
    // private:
        RegisterBank * mDevice;

        Eigen::Matrix<float, 3, 1> mScale;
        Eigen::Matrix<float, 3, 1> mMagneticField;
    };
}