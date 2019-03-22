#pragma once

#include "Algebra.hpp"

namespace icarus
{
    struct State
    {
        types::Vector3 position = types::Vector3::Zero();
        types::Vector3 velocity = types::Vector3::Zero();
        types::Vector3 acceleration = types::Vector3::Zero();
        types::Quaternion orientation = types::Quaternion::Identity();
        types::Vector3 angularVelocity = types::Vector3::Zero();
        types::Vector3 angularAcceleration = types::Vector3::Zero();
    };
}
