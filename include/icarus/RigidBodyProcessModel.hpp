#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <limits>

namespace icarus {
    template<typename T>
    struct RigidBodyProcessModel
    {
        using StateVector = Eigen::Matrix<T, 7, 1>;
        struct State
        {
            Eigen::Matrix<T, 3, 1> angularMomentum;
            Eigen::Quaternion<T> orientation;
        };
        static_assert(sizeof(State) == sizeof(StateVector), "The state vector does not correspond to the state.");

        StateVector operator()(StateVector const & stateVector, T timeStep)
        {
            auto & state = reinterpret_cast<State const &>(stateVector);

            StateVector ret;
            auto & newState = reinterpret_cast<State &>(ret);

            // law of conservation of angular momentum
            newState.angularMomentum = state.angularMomentum;

            // assuming identity moment of inertia tensor the angular velocity is equal in magnitude to angular momentum
            auto & angularVelocity = state.angularMomentum;

            T angularSpeed = angularVelocity.norm();
            if (std::abs(angularSpeed) > std::numeric_limits<T>::epsilon) {
                Eigen::Matrix<T, 3, 1> axis = angularVelocity / angularSpeed;
                T angle = angularSpeed * timeStep;

                Eigen::Quaternion<T> rotationChange = Eigen::AngleAxis<T>(angle, axis);

                newState.orientation = (rotationChange * state.orientation).normalized();
            } else {
                newState.orientation = state.orientation;
            }

            return ret;
        }
    };
}