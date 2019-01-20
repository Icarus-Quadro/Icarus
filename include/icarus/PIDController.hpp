#pragma once

#include "Controller.hpp"
#include "Thruster.hpp"

#include <cstddef>

namespace icarus
{
    struct PIDController : Controller
    {
        explicit PIDController(Thruster * thrusters, size_t size);
        void control(State const & current, State const & error) override;
        void p(types::Scalar value) { mP = value; }
        void i(types::Scalar value) { mI = value; }
        void d(types::Scalar value) { mD = value; }
    private:
        types::Scalar mP;
        types::Scalar mI;
        types::Scalar mD;
    };
}
