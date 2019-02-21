#pragma once

#include <Eigen/Dense>

namespace icarus {
    struct EllipsoidalCalibration
    {
        EllipsoidalCalibration(Eigen::Matrix<float, 3, 4> const & transformation);

        Eigen::Vector3f adjust(Eigen::Vector3f const & measurement) const;
    private:
        Eigen::Matrix<float, 3, 4> const mTransformation;
    };
}
