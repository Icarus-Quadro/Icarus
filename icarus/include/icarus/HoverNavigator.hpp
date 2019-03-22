#pragma once

#include "Navigator.hpp"

namespace icarus
{
    struct HoverNavigator : Navigator
    {
        State computeError(State const & current) override;
    };
}
