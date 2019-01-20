#include "KalmanFilter.hpp"

namespace icarus
{
    KalmanFilter::KalmanFilter()
    {
        
    }

    State KalmanFilter::state() const 
    {
        return State();
    }

    void KalmanFilter::integrateReadings(SensorReading const * readings, size_t size, types::Scalar timeDelta)
    {

    }
}
