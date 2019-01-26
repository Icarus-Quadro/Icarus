#pragma once

#include "AK8963.hpp"
#include "Algebra.hpp"
#include "Registers.hpp"

namespace icarus
{
    namespace registers::ak8963
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

        struct Measurements : Vector<int16_t, 3, Endian::little>
        {
            enum { address = 3 };

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

        struct SensitivityAdjustment : Vector<int8_t, 3>
        {
            enum { address = 16 };
        };
    }

    template<typename RegisterBank>
    AK8963<RegisterBank>::AK8963(RegisterBank * device) :
        mDevice(device)
    {}
    
    template<typename RegisterBank>
    void AK8963<RegisterBank>::initialize()
    {
        using namespace registers::ak8963;

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::fuseRomAccess;
            config.outputBit = OutputBitSetting::bits14;
        });

        mDevice->template read<SensitivityAdjustment>([this](auto & sensitivity) {
            auto scale = types::Vector3(sensitivity.x(), sensitivity.y(), sensitivity.z());
            mScale = ((scale / 256.0).array() + 1.0) * 0.6E-6;
        });

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::powerDown;
        });

        mDevice->template write<Control1>([](auto & config) {
            config.mode = OperationMode::continousMeasurement100Hz;
            config.outputBit = OutputBitSetting::bits14;
        });
    }
    
    template<typename RegisterBank>
    void AK8963<RegisterBank>::read()
    {
        using namespace registers::ak8963;

        mDevice->template read<Measurements>([this](auto const& data){
            mMagneticField = types::Vector3(data.x(), data.y(), data.z()).cwiseProduct(mScale);
        });
    }
}