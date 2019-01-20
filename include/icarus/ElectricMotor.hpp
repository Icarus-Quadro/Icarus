#pragma once

#include "Thruster.hpp"

namespace icarus
{
    struct ElectricMotor : Thruster
    {
        void throttle(types::Scalar fraction) override;
        types::Vector3 nominalForce() const override;
        types::Vector3 nominalTorque() const override;
    };
}