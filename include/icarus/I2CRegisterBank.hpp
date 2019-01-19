#pragma once

namespace icarus
{
    template<typename I2CBus>
    struct I2CRegisterBank
    {
        explicit I2CRegisterBank(I2CBus * bus, uint8_t deviceAddress) :
            mBus(bus),
            mDeviceAddress(deviceAddress)
        {}

        template<typename Register, typename F>
        void read(F readRegister)
        {
            uint8_t const registerAddress = Register::address;
            Register reg;
            mBus->transfer(mDeviceAddress, 
                reinterpret_cast<std::byte const *>(&registerAddress), 1, 
                reinterpret_cast<std::byte *>(&reg), sizeof(Register));
            readRegister(reg);
        }

        template<typename Register, typename F>
        void write(F writeRegister)
        {
            std::byte buffer[sizeof(Register) + 1];
            buffer[0] = static_cast<std::byte>(Register::address);
            auto & reg = *reinterpret_cast<Register *>(buffer + 1);
            writeRegister(reg);
            mBus->transfer(mDeviceAddress, buffer, sizeof(Register) + 1, nullptr, 0);
        }
    private:
        I2CBus * const mBus;
        uint8_t mDeviceAddress;
    };
}