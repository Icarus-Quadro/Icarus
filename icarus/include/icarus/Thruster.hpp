#pragma once

#include "Algebra.hpp"

namespace icarus
{
    struct Thruster
    {
        virtual ~Thruster() = default;
        virtual void throttle(types::Scalar fraction) = 0;
        virtual types::Vector3 nominalForce() const = 0;
        virtual types::Vector3 nominalTorque() const = 0;
    };
}
