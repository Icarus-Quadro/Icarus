#pragma once

#include "AK8963.hpp"
#include <Eigen/Dense>
#include <boost/endian/arithmetic.hpp>

namespace icarus
{
    namespace ak8963
    {
        enum class OperationMode : uint8_t
        {
            powerDown = 0b0000,
            singleMeasurement = 0b0001,
            continousMeasurement8Hz = 0b0010,
            continousMeasurement100Hz = 0b0110,
            externalTriggerMeasurement = 0b0100,
            selfTest = 0b1000,
            fuseRomAccess = 0b1111,
        };

        enum class OutputBitSetting : uint8_t
        {
            bits14 = 0,
            bits16 = 1,
        };

        struct Measurements
        {
            enum { address = 3 };
            
            Eigen::Matrix<boost::endian::little_int16_t, 3, 1> magneticField;
            uint8_t : 3;
            bool overflow : 1;
            OutputBitSetting outputBit : 1;
        };

        struct Control1
        {
            enum { address = 0xA };
            OperationMode mode : 4;
            OutputBitSetting outputBit : 1;
        };

        struct WhoAmI
        {
            enum { address = 0 };
            uint8_t data;
        };

        struct SensitivityAdjustment
        {
            enum { address = 16 };
            Eigen::Matrix<int8_t, 3, 1> adjustment;
        };
    }

    template<typename Delay, typename RegisterBank>
    AK8963<Delay, RegisterBank>::AK8963(RegisterBank * device) :
        mDevice(device)
    {}
    
    template<typename Delay, typename RegisterBank>
    void AK8963<Delay, RegisterBank>::initialize()
    {
        using namespace ak8963;

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::powerDown;
            config.outputBit = OutputBitSetting::bits14;
        });

        Delay::microseconds(100);

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::fuseRomAccess;
            config.outputBit = OutputBitSetting::bits14;
        });

        Delay::microseconds(100);

        mDevice->template read<SensitivityAdjustment>([this](auto & reg) {
            mScale = ((reg.adjustment.template cast<float>() / 256.0).array() + 1.0) * 0.6E-6;
        });

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::powerDown;
            config.outputBit = OutputBitSetting::bits14;
        });

        Delay::microseconds(100);

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::continousMeasurement100Hz;
            config.outputBit = OutputBitSetting::bits14;
        });

        Delay::microseconds(100);
    }
    
    template<typename Delay, typename RegisterBank>
    void AK8963<Delay, RegisterBank>::read()
    {
        using namespace ak8963;

        mDevice->template read<Measurements>([this](auto const& reg){
            mMagneticField = reg.magneticField.template cast<float>().cwiseProduct(mScale);
        });
    }
}