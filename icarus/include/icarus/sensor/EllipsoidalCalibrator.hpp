#pragma once

#include "EllipsoidalCalibration.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <iostream>
namespace icarus
{
    template<typename T>
    struct EllipsoidalCalibrator
    {
        EllipsoidalCalibrator(size_t sampleCount) :
            mSamples(10, sampleCount),
            mIndex(0)
        {}

        void addSample(Eigen::Matrix<T, 3, 1> const & p);

        EllipsoidalCalibration<T> computeCalibration(T norm) const;
    private:
        static Eigen::Matrix<T, 6, 1> computePositiveEigenVector(Eigen::Matrix<T, 6, 6> const & SS);
        static Eigen::Matrix<T, 10, 1> computeEllipsoidEquation(Eigen::Matrix<T, 10, 10> const & S);
        static Eigen::Matrix<T, 3, 4> computeEllipsoidTransformation(Eigen::Matrix<T, 10, 1> const & v, T norm);

        Eigen::Matrix<T, 10, Eigen::Dynamic> mSamples;
        size_t mIndex;
    };

    template<typename T>
    void EllipsoidalCalibrator<T>::addSample(Eigen::Matrix<T, 3, 1> const & p)
    {
        mSamples.col(mIndex) <<
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

        ++mIndex;
    }

    template<typename T>
    EllipsoidalCalibration<T> EllipsoidalCalibrator<T>::computeCalibration(T norm) const
    {
        Eigen::Matrix<T, 10, 10> S;
        // compute D * D.transpose() but in a way that is almost 3 times faster and uses 4 times less RAM
        S.setZero();
        S.template selfadjointView<Eigen::Lower>().rankUpdate(mSamples);
        S.template triangularView<Eigen::Upper>() = S.transpose();

        auto ellipsoidEquation = computeEllipsoidEquation(S);

        auto ellipsoid = computeEllipsoidTransformation(ellipsoidEquation, norm);

        return EllipsoidalCalibration<T>(ellipsoid);
    }

    template<typename T>
    Eigen::Matrix<T, 6, 1> EllipsoidalCalibrator<T>::computePositiveEigenVector(Eigen::Matrix<T, 6, 6> const & SS)
    {
        Eigen::Matrix<T, 6, 6> C;
        C.setZero();
        C.template block<3, 3>(0, 0).setOnes();
        C.diagonal() << -1, -1, -1, -4, -4, -4;

        Eigen::EigenSolver<Eigen::Matrix<T, 6, 6>> decomposition(C.lu().solve(SS));

        Eigen::Index maxCol;
        decomposition.eigenvalues().real().maxCoeff(&maxCol);
        Eigen::Matrix<T, 6, 1> v1 = decomposition.eigenvectors().col(maxCol).real();
        if (v1(1) < 0) {
            v1 = -v1;
        }
        return v1;
    }

    template<typename T>
    Eigen::Matrix<T, 10, 1> EllipsoidalCalibrator<T>::computeEllipsoidEquation(Eigen::Matrix<T, 10, 10> const & S)
    {
        auto S11 = S.template block<6, 6>(0, 0);
        auto S21 = S.template block<4, 6>(6, 0);
        auto S12 = S.template block<6, 4>(0, 6);//S21.transpose();
        auto S22 = S.template block<4, 4>(6, 6);
        Eigen::Matrix<T, 4, 6> const S22a = S22.inverse() * S21;
        Eigen::Matrix<T, 6, 6> const SS = S11 - S12 * S22a;


        Eigen::Matrix<T, 10, 1> v;
        auto v1 = v.template head<6>();
        auto v2 = v.template tail<4>();

        v1 = computePositiveEigenVector(SS);
        v2 = -S22a * v1;

        return v;
    }

    template<typename T>
    Eigen::Matrix<T, 3, 4> EllipsoidalCalibrator<T>::computeEllipsoidTransformation(Eigen::Matrix<T, 10, 1> const & v, T norm)
    {
        Eigen::Matrix<T, 3, 3> Q;
        Q << v[0], v[5], v[4],
             v[5], v[1], v[3],
             v[4], v[3], v[2];

        Eigen::Matrix<T, 3, 4> ret;
        auto Ainv = ret.template block<3, 3>(0, 0);
        auto B = ret.col(3);

        B = -Q.inverse() * v.template segment<3>(6);

        T scaling = norm / sqrt(B.transpose() * Q * B - v[9]);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T, 3, 3>> Qsolver(Q);
        Ainv.noalias() = scaling * Qsolver.operatorSqrt();
        B = -B;

        std::cout << ret << std::endl;

        return ret;
    }
}
