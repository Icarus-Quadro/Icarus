#pragma once

namespace icarus
{
    template<typename T>
    struct FlightModel
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
        static_assert(sizeof(Measurement) == sizeof(MeasurementVector), "The measurement vector does not correspond to the measurement.");

        struct MeasurementModel
        {
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

        struct ProcessModel
        {
            StateVector operator()(StateVector const & stateVector, T timeStep) const
            {
                auto & state = reinterpret_cast<State const &>(stateVector);

                StateVector ret;
                auto & newState = reinterpret_cast<State &>(ret);

                // law of conservation of angular momentum
                newState.angularMomentum = state.angularMomentum;

                // assuming identity moment of inertia tensor the angular velocity is equal in magnitude to angular momentum
                auto & angularVelocity = state.angularMomentum;

                T angularSpeed = angularVelocity.norm();
                if (std::abs(angularSpeed) > std::numeric_limits<T>::epsilon()) {
                    Eigen::Matrix<T, 3, 1> axis = angularVelocity / angularSpeed;
                    T angle = angularSpeed * timeStep;

                    Eigen::Quaternion<T> rotationChange;
                    rotationChange = Eigen::AngleAxis<T>(angle, axis);

                    newState.orientation = rotationChange * state.orientation;
                } else {
                    newState.orientation = state.orientation;
                }

                return ret;
            }

            Eigen::Matrix<T, 7, 7> noise() const
            {
                Eigen::Matrix<T, 7, 7> ret;
                ret.setZero();
                T rotVar = 0.0000001;
                T angMom = 0.1;
                ret.diagonal() << angMom, angMom, angMom, rotVar, rotVar, rotVar, rotVar;
                return ret;
            }
        };
    };
}