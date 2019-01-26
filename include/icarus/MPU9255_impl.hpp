#pragma once

#include "MPU9255.hpp"

#include "SensorReading.hpp"
#include "Registers.hpp"

#include <cmath>

namespace icarus
{
    namespace registers::mpu9255 {
        enum ActiveLevel : uint8_t
        {
            high,
            low,
        };

        enum PinMode : uint8_t 
        {
            pushPull,
            openDrain, 
        };

        enum class AccelerometerRange : uint8_t
        {
            g2,
            g4,
            g8,
            g16,
        };

        enum class GyroscopeRange : uint8_t
        {
            dps250,
            dps500,
            dps1000,
            dps2000,
        };

        enum class ClockSource : uint8_t
        {
            internal20MHz = 0,
            bestAvailable = 1,
            stopped = 7
        };

        struct SampleRateDivider
        {
            enum { address = 25 };
            uint8_t divider;
        };

        struct Configuration
        {
            uint8_t digitalLowPassFilter : 3;
            uint8_t fSync : 3;
            uint8_t fifoMode : 1;
        };

        struct AccelerometerConfiguration1
        {
            enum { address = 28 };
            uint8_t : 3;
            AccelerometerRange fullScaleSelect : 2;
            bool zAxisSelfTest : 1;
            bool yAxisSelfTest : 1;
            bool xAxisSelfTest : 1;
        };

        struct AccelerometerConfiguration2
        {
            enum { address = 29 };
            uint8_t digitalLowPassFilterConfiguration : 3;
            bool disableLowPassFilter : 1;
        };

        struct GyroscopeConfiguration
        {
            enum { address = 27 };
            uint8_t fChoiceB : 2;
            uint8_t : 1;
            GyroscopeRange fullScaleRange : 2;
            bool zAxisSelfTest : 1;
            bool yAxisSelfTest : 1;
            bool xAxisSelfTest : 1;
        };

        struct TemperatureRegister : Scalar<int16_t, Endian::big>
        {
            enum { address = 65 };
        };

        struct AccelerometerMeasurements : Vector<int16_t, 3, Endian::big>
        {
            enum { address = 59 };
        };

        struct GyroscopeMeasurements : Vector<int16_t, 3, Endian::big>
        {
            enum { address = 67 };
        };

        struct InterruptBypassConfiguration
        {
            enum { address = 55 };

            uint8_t : 1;
            bool enableBypass : 1;
            bool enableFsyncInterrupt : 1;
            ActiveLevel fsyncActiveLevel : 1;
            bool clearInterruptOnRead : 1;
            bool latchInterrputPin : 1;
            PinMode interruptPinMode : 1;
            ActiveLevel interruptPinLevel : 1;
        };

        struct PowerManagement1
        {
            enum { address = 107 };
            ClockSource clockSelect : 3;
            bool powerDownProportionalToAbsoulteTemperatureVoltageGenerator : 1;
            bool gyroscopeStandby : 1;
            bool cycle : 1;
            bool sleep : 1;
            bool hardReset : 1;
        };
    }

    template<typename RegisterBank>
    MPU9255<RegisterBank>::MPU9255(RegisterBank * device) :
        mDevice(device)
    {}
    
    template<typename RegisterBank>
    void MPU9255<RegisterBank>::initialize()
    {
        using namespace registers::mpu9255;

        mDevice->template write<InterruptBypassConfiguration>([](auto & config) {
            config.enableBypass = true;
        });

        mDevice->template write<AccelerometerConfiguration1>([](auto & config) {
            config.fullScaleSelect = AccelerometerRange::g8;
        });

        constexpr types::Scalar STANDARD_GRAVITY =  9.80665;
        mAccelerationScale = STANDARD_GRAVITY / 4096.0;

        constexpr types::Scalar RADIANS_PER_DEGREE = (2 * M_PI) / 360.0;
        mAngluarVelocityScale = 1.0 / 131.0 * RADIANS_PER_DEGREE;
    }
    
    template<typename RegisterBank>
    void MPU9255<RegisterBank>::read()
    {
        using namespace registers::mpu9255;

        mDevice->template read<TemperatureRegister>([this](auto const& data){
            constexpr types::Scalar KELVINS_PER_LSB = 1.0 / 333.87;
            constexpr types::Scalar OFFSET_FROM_ABSOLUTE_ZERO = 294.15;
            mTemperature = static_cast<types::Scalar>(data.value()) * KELVINS_PER_LSB + OFFSET_FROM_ABSOLUTE_ZERO;
        });
        
        mDevice->template read<AccelerometerMeasurements>([this](auto const& data){
            mAcceleration = types::Vector3(data.x(), data.y(), data.z()) * mAccelerationScale;
        });

        mDevice->template read<GyroscopeMeasurements>([this](auto const& data){
            mAngularVelocity = types::Vector3(data.x(), data.y(), data.z()) * mAngluarVelocityScale;
        });
    }
}