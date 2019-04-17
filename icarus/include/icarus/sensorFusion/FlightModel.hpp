#pragma once

#include "../sensor/GasModel.hpp"

namespace icarus
{
    template<typename T>
    struct FlightModel
    {
        using StateVector = Eigen::Matrix<T, 16, 1>;
        struct State
        {
            explicit State(StateVector & vector) :
                mVector(vector)
            {}

            auto orientation()
            {
                return Eigen::Map<Eigen::Quaternion<T>>(mVector.data());
            }

            auto angularMomentum()
            {
                return mVector.template segment<3>(4);
            }

            auto position()
            {
                return mVector.template segment<3>(7);
            }

            auto velocity()
            {
                return mVector.template segment<3>(10);
            }

            auto acceleration()
            {
                return mVector.template segment<3>(13);
            }

            void reset()
            {
                orientation().setIdentity();
                angularMomentum().setZero();
                position().setZero();
                velocity().setZero();
                acceleration().setZero();
            }
        private:
            StateVector & mVector;
        };

        using MeasurementVector = Eigen::Matrix<T, 10, 1>;
        struct Measurement {
            explicit Measurement(MeasurementVector & vector) :
                mVector(vector)
            {}

            auto angularVelocity()
            {
                return mVector.template segment<3>(0);
            }

            auto magneticField()
            {
                return mVector.template segment<3>(3);
            }

            auto acceleration()
            {
                return mVector.template segment<3>(6);
            }

            auto & pressure()
            {
                return mVector[9];
            }
        private:
            MeasurementVector & mVector;
        };

        struct MeasurementModel
        {
            MeasurementVector operator()(StateVector const & stateVector) const
            {
                auto state = State(const_cast<StateVector&>(stateVector));

                MeasurementVector ret;
                auto measurement = Measurement(ret);

                // transform angular velocity from world space to body space
                // transform vector by the inverse of body orientation
                auto rotation = state.orientation().conjugate().toRotationMatrix();

                measurement.angularVelocity() = rotation * state.angularMomentum();
                measurement.magneticField() = rotation * mReferenceMagneticField;
                measurement.acceleration() = rotation * (state.acceleration() + Eigen::Matrix<T, 3, 1>(0, 0, 9.81));

                constexpr GasModel<float> gm;
                measurement.pressure() = gm.pressure(state.position().z() + mReferenceAltitude);

                return ret;
            }

            void referenceMagneticField(Eigen::Matrix<T, 3, 1> const & value)
            {
                mReferenceMagneticField = value;
            }

            void referencePressure(T value)
            {
                constexpr GasModel<float> gm;
                mReferenceAltitude = gm.altitude(value);
            }

        private:
            Eigen::Matrix<T, 3, 1> mReferenceMagneticField;
            T mReferenceAltitude;
        };

        struct ProcessModel
        {
            StateVector operator()(StateVector const & stateVector, T timeStep) const
            {
                auto state = State(const_cast<StateVector&>(stateVector));

                StateVector ret;
                auto newState = State(ret);

                // law of conservation of angular momentum
                newState.angularMomentum() = state.angularMomentum();

                // assuming identity moment of inertia tensor the angular velocity is equal in magnitude to angular momentum
                auto angularVelocity = state.angularMomentum();

                T angularSpeed = angularVelocity.norm();
                if (std::abs(angularSpeed) > std::numeric_limits<T>::epsilon()) {
                    Eigen::Matrix<T, 3, 1> axis = angularVelocity / angularSpeed;
                    T angle = angularSpeed * timeStep;

                    Eigen::Quaternion<T> rotationChange;
                    rotationChange = Eigen::AngleAxis<T>(angle, axis);

                    newState.orientation() = rotationChange * state.orientation();
                } else {
                    newState.orientation() = state.orientation();
                }

                newState.position() = state.position() * 0.999 + state.velocity() * timeStep + state.acceleration() * timeStep * timeStep / 2;
                newState.velocity() = state.velocity() * 0.999 + state.acceleration() * timeStep;
                newState.acceleration() = state.acceleration();

                return ret;
            }

            Eigen::Matrix<T, 16, 16> noise() const
            {
                Eigen::Matrix<T, 16, 16> ret;
                ret.setZero();
                T rotVar = 0.0000001;
                T angMom = 0.1;
                T pos = 0.000001;
                T vel = 0.0001;
                T acc = 0.01;
                ret.diagonal() <<
                    rotVar, rotVar, rotVar, rotVar,
                    angMom, angMom, angMom,
                    pos, pos, pos,
                    vel, vel, vel,
                    acc, acc, acc;

                return ret;
            }
        };
    };
}
