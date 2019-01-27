#pragma once

#include "Algebra.hpp"

namespace icarus
{
    template<typename RegisterBank>
    struct BMP180
    {
        explicit BMP180(RegisterBank * device);
        void initialize();
        
        void startTemperatureRead();
        void readTemperature();

        void startPressureRead();
        void readPressure();

        types::Scalar altitude() const
        {
            
        }

        types::Scalar pressure() const
        {
            
        }
    private:
        RegisterBank * mDevice;

        struct {
            uint16_t ac1, ac2, ac3;
            int16_t ac4, ac5, ac6;
            uint16_t b1, b2, mb, mc, md;
        } mCallibration;
    };
}