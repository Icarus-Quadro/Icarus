#include "EllipsoidalCalibrator.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

using namespace Eigen;

template<typename Derived>
Matrix<typename Derived::Scalar, 10, 10> multiplyDesignMatrix(MatrixBase<Derived> const & samples)
{
    using Scalar = typename Derived::Scalar;

    auto const size = samples.cols();
    Matrix<Scalar, 10, Dynamic> D(10, size);

    for (int i = 0; i < size; ++i) {
        auto const & p = samples.col(i);
        D.col(i) <<
            p.x() * p.x(),
            p.y() * p.y(),
            p.z() * p.z(),
            2 * p.y() * p.z(),
            2 * p.x() * p.z(),
            2 * p.x() * p.y(),
            2 * p.x(),
            2 * p.y(),
            2 * p.z(),
            1;
    }

    Matrix<Scalar, 10, 10> S;
    // compute D * D.transpose() but in a way that is almost 3 times faster and uses 4 times less RAM
    S.setZero();
    S.template selfadjointView<Lower>().rankUpdate(D);
    S.template triangularView<Upper>() = S.transpose();
    return S;
}

template<typename Derived>
Matrix<typename Derived::Scalar, 6, 1> computePositiveEigenVector(MatrixBase<Derived> const & SS)
{
    using Scalar = typename Derived::Scalar;
    Matrix<Scalar, 6, 6> C;
    C.setZero();
    C.template block<3, 3>(0, 0).setOnes();
    C.diagonal() << -1, -1, -1, -4, -4, -4;

    EigenSolver<Matrix<Scalar, 6, 6>> decomposition(C.lu().solve(SS));

    Index maxCol;
    auto max = decomposition.eigenvalues().real().maxCoeff(&maxCol);
    Matrix<Scalar, 6, 1> v1 = decomposition.eigenvectors().col(maxCol).real();
    if (v1(1) < 0) {
        v1 = -v1;
    }
    return v1;
}

template<typename Scalar>
Matrix<Scalar, 10, 1> computeEllipsoidEquation(Matrix<Scalar, 10, 10> const & S)
{
    auto S11 = S.template block<6, 6>(0, 0);
    auto S21 = S.template block<4, 6>(6, 0);
    auto S12 = S21.transpose();
    auto S22 = S.template block<4, 4>(6, 6).template selfadjointView<Lower>();
    Matrix<Scalar, 4, 6> const S22a = S22.ldlt().solve(S21);
    Matrix<Scalar, 6, 6> const SS = S11 - S12 * S22a;


    Matrix<Scalar, 10, 1> v;
    auto v1 = v.template head<6>();
    auto v2 = v.template tail<4>();

    v1 = computePositiveEigenVector(SS);
    v2 = -S22a * v1;

    return v;
}

template<typename Scalar>
Matrix<Scalar, 3, 4> computeEllipsoidTransformation(Matrix<Scalar, 10, 1> const & v, Scalar norm)
{
    Matrix<Scalar, 3, 3> Q;
    Q << v[0], 0, 0, v[5], v[1], 0, v[4], v[3], v[2];

    Matrix<Scalar, 3, 4> ret;
    auto Ainv = ret.template block<3, 3>(0, 0);
    auto B = ret.col(3);

    B = -Q.template selfadjointView<Lower>().ldlt().solve(v.template segment<3>(6));

    Scalar scaling = norm / sqrt(B.transpose() * Q * B - v[9]);

    SelfAdjointEigenSolver<Matrix<Scalar, 3, 3>> Qsolver(Q);
    Ainv.noalias() = scaling * Qsolver.operatorSqrt();
    B = -B;

    return ret;
}

namespace icarus
{
    EllipsoidalCalibrator::EllipsoidalCalibrator(size_t sampleCount) :
        mSamples(sampleCount)
    {}

    EllipsoidalCalibration EllipsoidalCalibrator::computeCalibration(float norm) const
    {
        Map<Matrix<float, 3, Dynamic> const> designMatrix(
            reinterpret_cast<float const *>(mSamples.data()),
            3,
            mSamples.size()
        );

        auto S = multiplyDesignMatrix(designMatrix);
        auto ellipsoidEquation = computeEllipsoidEquation(S);

        auto ellipsoid = computeEllipsoidTransformation(ellipsoidEquation, norm);

        return EllipsoidalCalibration(ellipsoid);
    }
}
