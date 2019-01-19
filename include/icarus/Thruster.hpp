#pragma once

#include "Algebra.hpp"

namespace icarus
{
    struct Thruster
    {
        virtual ~Thruster() = default;
        virtual void throttle(Scalar fraction) = 0;
        virtual Vector3 nominalForce() const = 0;
        virtual Vector3 nominalTorque() const = 0;
    };
}
