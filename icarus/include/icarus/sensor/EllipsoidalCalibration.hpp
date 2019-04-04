#pragma once

#include <Eigen/Dense>

namespace icarus {
    template <typename T>
    struct EllipsoidalCalibration
    {
        explicit EllipsoidalCalibration() :
            mTransformation(Eigen::Matrix<float, 3, 4>::Identity())
        {}

        explicit EllipsoidalCalibration(Eigen::Matrix<T, 3, 4> const & transformation) :
            mTransformation(transformation)
        {}

        void transformAxes(Eigen::Matrix<T, 3, 3> const & transform)
        {
            mTransformation.template block<3, 3>(0, 0) = transform * mTransformation.template block<3, 3>(0, 0);
        }

        void addOffset(Eigen::Matrix<T, 3, 1> const & offset)
        {
            mTransformation.col(3) += offset;
        }

        Eigen::Matrix<T, 3, 1> adjust(Eigen::Matrix<T, 3, 1> const & measurement) const
        {
            return mTransformation.template block<3, 3>(0, 0)
                * (measurement + mTransformation.col(3));
        }

        Eigen::Matrix<T, 3, 4> const & transformation() const
        {
            return mTransformation;
        }
    private:
        Eigen::Matrix<T, 3, 4> mTransformation;
    };
}
