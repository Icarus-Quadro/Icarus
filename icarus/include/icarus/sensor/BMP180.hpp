#pragma once

#include "Algebra.hpp"

namespace icarus
{
    template<typename RegisterBank>
    struct BMP180
    {
        explicit BMP180(RegisterBank * device);
        void initialize();

        void read();

        float temperature() const
        {
            return mTemperature;
        }

        float pressure() const
        {
            return mPressure;
        }
    private:
        void startTemperatureRead();
        void readTemperature();

        void startPressureRead();
        void readPressure();

        RegisterBank * mDevice;

        struct {
            int16_t ac1, ac2, ac3;
            uint16_t ac4, ac5, ac6;
            int16_t b1, b2, mb, mc, md;
        } mCallibration;

        int32_t mB5;

        int mPeriodsToTemperatureUpdate;
        float mTemperature;
        float mPressure;
    };
}