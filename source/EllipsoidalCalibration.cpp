#include "EllipsoidalCalibration.hpp"

using namespace Eigen;

namespace icarus {
    EllipsoidalCalibration::EllipsoidalCalibration() :
        mTransformation(Matrix<float, 3, 4>::Identity())
    {}

    EllipsoidalCalibration::EllipsoidalCalibration(Matrix<float, 3, 4> const & transformation) :
        mTransformation(transformation)
    {}

    Vector3f EllipsoidalCalibration::adjust(Vector3f const & measurement) const
    {
        return mTransformation.block<3, 3>(0, 0).selfadjointView<Lower>()
            * (measurement + mTransformation.col(3));
    }
}
