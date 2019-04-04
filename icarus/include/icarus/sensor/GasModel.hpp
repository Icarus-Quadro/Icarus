#pragma once

#include <cmath>

namespace icarus
{
    template<typename T>
    struct GasModel
    {
        explicit constexpr GasModel(T temperature = 288.15, T specificGasConstant = 287.058) :
            mScaleHeight(specificGasConstant * temperature / standardGravity)
        {}

        constexpr T altitude(T pressure) const
        {
            return mScaleHeight * std::log(standardPressure / pressure);
        }

        constexpr T pressure(T altitude) const
        {
            return standardPressure * std::exp(-altitude / mScaleHeight);
        }
    private:
        T mScaleHeight;
        static constexpr T standardPressure = 101325;
        static constexpr T standardGravity = 9.81;
    };
}
