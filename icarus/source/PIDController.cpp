#include "PIDController.hpp"

namespace icarus
{
    PIDController::PIDController(Thruster * thrusters, size_t size) :
        mP(1.0),
        mI(0.0),
        mD(1.0)
    {

    }

    void PIDController::control(State const & current, State const & error)
    {

    }
}
