#include "Aircraft.hpp"

#include "SensorFusion.hpp"
#include "Navigator.hpp"
#include "Controller.hpp"

namespace icarus
{
    Aircraft::Aircraft(Controller * controller, SensorFusion * sensorFusion) :
        mController(controller),
        mSensorFusion(sensorFusion),
        mNavigator(nullptr)
    {}

    void Aircraft::fly()
    {
        if (mNavigator) {
            auto const state = mSensorFusion->state();
            auto const error = mNavigator->computeError(state);
            mController->control(state, error);
        }
    }
}
