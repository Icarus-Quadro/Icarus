#pragma once

#include "MPU9255.hpp"
#include <boost/endian/arithmetic.hpp>
#include <Eigen/Dense>

namespace icarus
{
    namespace mpu9255 {
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
            enum { address = 26 };
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

        struct TemperatureRegister
        {
            enum { address = 65 };
            boost::endian::big_int16_t temperature;
        };

        struct AccelerometerMeasurements
        {
            enum { address = 59 };
            Eigen::Matrix<boost::endian::big_int16_t, 3, 1> acceleration;
        };

        struct GyroscopeMeasurements
        {
            enum { address = 67 };
            Eigen::Matrix<boost::endian::big_int16_t, 3, 1> angularVelocity;
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

        struct WhoAmI
        {
            enum { address = 117 };
            uint8_t id;
        };
    }

    template<typename RegisterBank>
    MPU9255<RegisterBank>::MPU9255(RegisterBank * device) :
        mDevice(device),
        mGyroscopeScaleRange(0)
    {}

    template<typename RegisterBank>
    void MPU9255<RegisterBank>::initialize()
    {
        using namespace mpu9255;

        mDevice->template read<WhoAmI>([](auto & reg){
            if (reg.id != 0x73) {
                std::stringstream ss;
                    ss << "Unrecognized device on the bus. Expected MPU9255: id: "
                    << std::hex << 0x73 << ", got: " << int(reg.id);
                throw std::runtime_error(ss.str());
            }
        });

        mDevice->template write<AccelerometerConfiguration1>([](auto & config) {
            config.fullScaleSelect = AccelerometerRange::g8;
        });

        mDevice->template write<Configuration>([](auto & config) {
            config.digitalLowPassFilter = 2;
            config.fSync = 0;
            config.fifoMode = 0;
        });

        mDevice->template write<GyroscopeConfiguration>([](auto & config) {
            config.fChoiceB = 0;
            config.fullScaleRange = mpu9255::GyroscopeRange::dps250;
            config.zAxisSelfTest = false;
            config.yAxisSelfTest = false;
            config.xAxisSelfTest = false;
        });

        mDevice->template write<PowerManagement1>([](auto & config) {
            config.clockSelect = mpu9255::ClockSource::bestAvailable;
            config.powerDownProportionalToAbsoulteTemperatureVoltageGenerator = false;
            config.gyroscopeStandby = false;
            config.cycle = false;
            config.sleep = false;
            config.hardReset = false;
        });

        constexpr float STANDARD_GRAVITY =  9.80665;
        mAccelerationScale = STANDARD_GRAVITY / 4096.0;
    }

    template<typename RegisterBank>
    void MPU9255<RegisterBank>::i2cBypass(bool enable)
    {
        using namespace mpu9255;

        mDevice->template write<InterruptBypassConfiguration>([enable](auto & config) {
            config.enableBypass = enable;
        });
    }

    template<typename RegisterBank>
    void MPU9255<RegisterBank>::read()
    {
        using namespace mpu9255;

        mDevice->template read<TemperatureRegister>([this](auto const& reg){
            constexpr float KELVINS_PER_LSB = 1.0 / 333.87;
            constexpr float OFFSET_FROM_ABSOLUTE_ZERO = 294.15;
            mTemperature = static_cast<float>(reg.temperature) * KELVINS_PER_LSB + OFFSET_FROM_ABSOLUTE_ZERO;
        });

        mDevice->template read<AccelerometerMeasurements>([this](auto const& reg){
            mAcceleration = reg.acceleration.template cast<float>() * mAccelerationScale;
        });

        int newRange;
        mDevice->template read<GyroscopeMeasurements>([this, &newRange](auto const& reg){
            constexpr float radiansPerDegree = (2 * M_PI) / 360.0;
            constexpr float baseSensitivity = 1.0 / 131.0 * radiansPerDegree;

            float const sensitivity = (1 << mGyroscopeScaleRange) * baseSensitivity;
            mAngularVelocity = reg.angularVelocity.template cast<float>() * sensitivity;

            constexpr int highBound = 23170; // reading above which we attempt to increase full scale select
            constexpr int lowBound = 8192; // reading below which we attempt to decrease full scale select
            int const maximum = reg.angularVelocity.template cast<int>().cwiseAbs().maxCoeff();
            if (maximum > highBound && mGyroscopeScaleRange < 4) {
                newRange = mGyroscopeScaleRange + 1;
            } else if (maximum < lowBound && mGyroscopeScaleRange > 0) {
                newRange = mGyroscopeScaleRange - 1;
            } else {
                newRange = mGyroscopeScaleRange;
            }
        });

        if (newRange != mGyroscopeScaleRange) {
            mDevice->template write<mpu9255::GyroscopeConfiguration>([=](auto & config) {
                config.fChoiceB = 0;
                config.fullScaleRange = (mpu9255::GyroscopeRange) newRange;
                config.zAxisSelfTest = false;
                config.yAxisSelfTest = false;
                config.xAxisSelfTest = false;
            });

            mGyroscopeScaleRange = newRange;
        }
    }
}