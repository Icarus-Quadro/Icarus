#pragma once

namespace icarus
{
    class Controller;
    class Navigator;
    class SensorFusion;

    struct Aircraft
    {
        explicit Aircraft(Controller * controller, SensorFusion * sensorFusion);
        void navigator(Navigator * value) { mNavigator = value; }
        void fly();
    private:
        Controller * const mController;
        SensorFusion * const mSensorFusion;
        Navigator * mNavigator;
    };
}
