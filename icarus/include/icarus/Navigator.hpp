#pragma once

#include "State.hpp"

namespace icarus
{
    struct Navigator
    {
        virtual ~Navigator() = default;
        virtual State computeError(State const & current) = 0;
    };
}
