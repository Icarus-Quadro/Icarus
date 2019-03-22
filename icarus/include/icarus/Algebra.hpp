#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace icarus::types
{
    using Scalar = float;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;
}
