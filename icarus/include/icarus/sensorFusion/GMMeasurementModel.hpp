#pragma once

namespace icarus
{
    template<typename T>
    struct GMMeasurementModel
    {
        using StateVector = Eigen::Matrix<T, 7, 1>;
        struct State
        {
            Eigen::Matrix<T, 3, 1> angularMomentum;
            Eigen::Quaternion<T> orientation;
        };
        static_assert(sizeof(State) == sizeof(StateVector), "The state vector does not correspond to the state.");

        using MeasurementVector = Eigen::Matrix<T, 6, 1>;
        struct Measurement {
            Eigen::Matrix<T, 3, 1> angularVelocity;
            Eigen::Matrix<T, 3, 1> magneticField;
        };

        MeasurementVector operator()(StateVector const & stateVector) const
        {
            auto & state = reinterpret_cast<State const &>(stateVector);

            MeasurementVector ret;
            auto & measurement = reinterpret_cast<Measurement &>(ret);

            // transform angular velocity from world space to body space
            // transform vector by the inverse of body orientation
            auto rotation = state.orientation.conjugate().toRotationMatrix();

            measurement.angularVelocity = rotation * state.angularMomentum;
            measurement.magneticField = rotation * mReferenceMagneticField;

            return ret;
        }

        void referenceMagneticField(Eigen::Matrix<T, 3, 1> const & value)
        {
            mReferenceMagneticField = value;
        }
    private:
        Eigen::Matrix<T, 3, 1> mReferenceMagneticField;
    };
}
