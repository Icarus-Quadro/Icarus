#include "HoverNavigator.hpp"

#include "State.hpp"

namespace icarus
{
    State HoverNavigator::computeError(State const & current)
    {
        State ret;
        ret.orientation = current.orientation.conjugate();
        ret.velocity = -current.velocity;
        return ret;
    }
}
