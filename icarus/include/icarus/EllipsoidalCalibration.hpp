#pragma once

#include <Eigen/Dense>

namespace icarus {
    struct EllipsoidalCalibration
    {
        // EllipsoidalCalibration(EllipsoidalCalibration const &) = default;
        explicit EllipsoidalCalibration();
        explicit EllipsoidalCalibration(Eigen::Matrix<float, 3, 4> const & transformation);


        Eigen::Vector3f adjust(Eigen::Vector3f const & measurement) const;

        // EllipsoidalCalibration & operator=(EllipsoidalCalibration const &) = default;
    private:
        Eigen::Matrix<float, 3, 4> mTransformation;
    };
}
