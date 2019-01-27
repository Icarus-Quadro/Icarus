#pragma once

#include "BMP180.hpp"
#include <boost/endian/arithmetic.hpp>

namespace icarus
{
    namespace registers::bmp180
    {
        struct Callibration
        {
            enum { address = 0xAA };

            boost::endian::big_uint16_t ac1, ac2, ac3;
            boost::endian::big_int16_t ac4, ac5, ac6;
            boost::endian::big_uint16_t b1, b2, mb, mc, md;
        };

        enum class MeasurementControl : uint8_t
        {
            pressure = 0b0100,
            temperature = 0b1110,
        };

        enum class Oversampling : uint8_t
        {
            times1,
            times2,
            times4,
            times8,
        };

        struct Control
        {
            enum { address = 0xF4 };

            MeasurementControl measurementControl : 5;
            bool startOfConversion : 1;
            Oversampling oversampling : 2;
        };

        struct TemperatureReading
        {
            enum { address = 0xF6 };

            boost::endian::big_int16_t temperature;
        };

        struct PressureReading
        {
            enum { address = 0xF6 };
            boost::endian::big_uint24_t pressure;
        };
    }
    
    template<typename RegisterBank>
    BMP180<RegisterBank>::BMP180(RegisterBank * device) :
        mDevice(device)
    {}

    template<typename RegisterBank>
    void BMP180<RegisterBank>::initialize()
    {
        mDevice->template read<registers::bmp180::Callibration>([this](auto const & cal) {
            mCallibration.ac1 = cal.ac1;
            mCallibration.ac2 = cal.ac2;
            mCallibration.ac3 = cal.ac3;
            mCallibration.ac4 = cal.ac4;
            mCallibration.ac5 = cal.ac5;
            mCallibration.ac6 = cal.ac6;
            mCallibration.b1 = cal.b1;
            mCallibration.b2 = cal.b2;
            mCallibration.mb = cal.mb;
            mCallibration.mc = cal.mc;
            mCallibration.md = cal.md;
        });
    }
    
    template<typename RegisterBank>
    void BMP180<RegisterBank>::startTemperatureRead()
    {
        using namespace registers::bmp180;
        mDevice->template write<Control>([this](auto & reg) {
            reg.measurementControl = MeasurementControl::temperature;
            reg.startOfConversion = true;
            reg.oversampling = Oversampling::times1;
        });
    }
    
    template<typename RegisterBank>
    void BMP180<RegisterBank>::readTemperature()
    {
        mDevice->template read<registers::bmp180::TemperatureReading>([this](auto const & reg) {
            
        });
    }

    template<typename RegisterBank>
    void BMP180<RegisterBank>::startPressureRead()
    {

    }
    
    template<typename RegisterBank>
    void BMP180<RegisterBank>::readPressure()
    {
        
    }
}