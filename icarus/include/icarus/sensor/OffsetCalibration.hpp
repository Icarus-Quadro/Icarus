#pragma once

#include <Eigen/Dense>

namespace icarus
{
    template<typename T, size_t N>
    struct OffsetCalibration
    {
        explicit OffsetCalibration()
        {
            mOffset.setZero();
        }

        explicit OffsetCalibration(Eigen::Matrix<T, N, 1> const & offset) :
            mOffset(-offset)
        {}

        Eigen::Matrix<T, N, 1> adjust(Eigen::Matrix<T, N, 1> const & reading) const
        {
            return reading + mOffset;
        }
    private:
        Eigen::Matrix<T, N, 1> mOffset;
    };
}
