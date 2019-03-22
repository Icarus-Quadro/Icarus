#pragma once

namespace icarus
{
    struct State;

    struct Controller
    {
        virtual ~Controller() = default;
        virtual void control(State const & current, State const & error) = 0;
    };
}
