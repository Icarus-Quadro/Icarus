#pragma once

#include "BMP180.hpp"

#include <boost/endian/arithmetic.hpp>
#include <limits>

namespace icarus
{
    namespace registers::bmp180
    {
        struct Callibration
        {
            enum { address = 0xAA };

            boost::endian::big_int16_t ac1, ac2, ac3;
            boost::endian::big_uint16_t ac4, ac5, ac6;
            boost::endian::big_int16_t b1, b2, mb, mc, md;
        };

        enum class MeasurementControl : uint8_t
        {
            pressure = 0b10100,
            temperature = 0b01110,
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
            boost::endian::big_uint16_t temperature;
        };

        struct PressureReading
        {
            enum { address = 0xF6 };
            boost::endian::big_uint24_t pressure;
        };

        struct ChipId
        {
            enum { address = 0xD0 };
            uint8_t id;
        };
    }

    template<typename RegisterBank>
    BMP180<RegisterBank>::BMP180(RegisterBank * device) :
        mDevice(device),
        mPeriodsToTemperatureUpdate(0),
        mTemperature(std::numeric_limits<float>::quiet_NaN()),
        mPressure(std::numeric_limits<float>::quiet_NaN())
    {}

    template<typename RegisterBank>
    void BMP180<RegisterBank>::initialize()
    {
        using namespace registers::bmp180;

        mDevice->template read<ChipId>([](auto & reg){
            if (reg.id != 0x55) {
                throw std::runtime_error("Unrecognized bmp180 ID");
            }
        });

        mDevice->template read<Callibration>([this](auto const & cal) {
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

        startTemperatureRead();
    }


    template<typename RegisterBank>
    void BMP180<RegisterBank>::read()
    {
        if (mPeriodsToTemperatureUpdate == 0) {
            readTemperature();
            mPeriodsToTemperatureUpdate = 20;
        } else {
            readPressure();
        }

        --mPeriodsToTemperatureUpdate;

        if (mPeriodsToTemperatureUpdate == 0) {
            startTemperatureRead();
        } else {
            startPressureRead();
        }
    }

    template<typename RegisterBank>
    void BMP180<RegisterBank>::startTemperatureRead()
    {
        using namespace registers::bmp180;

        mDevice->template write<Control>([this](auto & reg) {
            reg.measurementControl = MeasurementControl::temperature;
            reg.oversampling = Oversampling::times1;
            reg.startOfConversion = true;
        });
    }

    template<typename RegisterBank>
    void BMP180<RegisterBank>::readTemperature()
    {
        mDevice->template read<registers::bmp180::TemperatureReading>([this](auto const & reg) {
            int32_t t = reg.temperature;
            int32_t x1 = ((t - int32_t(mCallibration.ac6)) * int32_t(mCallibration.ac5)) >> 15;
            int32_t x2 = (int32_t(mCallibration.mc) << 11) / (x1 + int32_t(mCallibration.md));
            mB5 = x1 + x2;
            mTemperature = float(mB5 + 8) / (10 << 4) + 273.15;
        });
    }

    template<typename RegisterBank>
    void BMP180<RegisterBank>::startPressureRead()
    {
        using namespace registers::bmp180;

        mDevice->template write<Control>([this](auto & reg) {
            reg.measurementControl = MeasurementControl::pressure;
            reg.oversampling = Oversampling::times2;
            reg.startOfConversion = true;
        });
    }

    template<typename RegisterBank>
    void BMP180<RegisterBank>::readPressure()
    {
        using namespace registers::bmp180;

        mDevice->template read<PressureReading>([this](auto const & reg) {
            // BEWARE!! Here be dragons
            // The following piece of code is based on BMP180 datasheet
            constexpr uint32_t oss = 1;
            uint32_t up = uint32_t(reg.pressure) >> (8 - oss);
            int32_t b6 = mB5 - 4000;
            int32_t x1 = (int32_t(mCallibration.b2) * ((b6 * b6) >> 12)) >> 11;
            int32_t x2 = (int32_t(mCallibration.ac2) * b6) >> 11;
            int32_t x3 = x1 + x1;
            int32_t b3 = (((int32_t(mCallibration.ac1) * 4 + x3) << oss) + 2) / 4;
            x1 = (int32_t(mCallibration.ac3) * b6) >> 13;
            x2 = (int32_t(mCallibration.b1) * ((b6 *b6) >> 12)) >> 16;
            x3 = ((x1 + x2) + 2) >> 2;
            uint32_t b4 = (uint32_t(mCallibration.ac4) * uint32_t(x3 + 32768)) >> 15;
            uint32_t b7 = (up - uint32_t(b3)) * (50000 >> oss);
            uint32_t p = (b7 / b4) * 2;
            x1 = (p >> 8) * (p >> 8);
            x1 = (x1 * 3038) >> 16;
            x2 = (-7357 * int32_t(p)) >> 16;
            p = p + ((x1 + x2 + 3791) >> 4);
            mPressure = float(p);
        });
    }
}