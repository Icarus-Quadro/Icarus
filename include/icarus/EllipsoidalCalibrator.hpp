#pragma once

#include "EllipsoidalCalibration.hpp"

#include <Eigen/Dense>
#include <vector>

namespace icarus
{
    struct EllipsoidalCalibrator
    {
        EllipsoidalCalibrator(size_t sampleCount);

        void addSample(Eigen::Vector3f const & sample)
        {
            mSamples.push_back(sample);
        }

        EllipsoidalCalibration computeCalibration(float norm) const;
    private:
        std::vector<Eigen::Matrix<float, 3, 1>> mSamples;
    };
}
