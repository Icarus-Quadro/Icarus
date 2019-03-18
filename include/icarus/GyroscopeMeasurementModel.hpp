#pragma once

namespace icarus
{
    template<typename T>
    struct GyroscopeMeasurementModel
    {
        using StateVector = Eigen::Matrix<T, 7, 1>;
        struct State
        {
            Eigen::Matrix<T, 3, 1> angularMomentum;
            Eigen::Quaternion<T> orientation;
        };
        static_assert(sizeof(State) == sizeof(StateVector), "The state vector does not correspond to the state.");

        using MeasurementVector = Eigen::Matrix<T, 3, 1>;
        struct Measurement {
            Eigen::Matrix<T, 3, 1> angularVelocity;
        };

        MeasurementVector operator()(StateVector const & stateVector) const
        {
            auto & state = reinterpret_cast<State const &>(stateVector);

            MeasurementVector ret;
            auto & measurement = reinterpret_cast<Measurement &>(ret);

            measurement.angularVelocity = state.angularMomentum;

            return ret;
        }
    };
}
