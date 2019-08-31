#pragma once

#include <gmock/gmock-matchers.h>

#include <Eigen/Geometry>

#include <type_traits>

template <typename Matrix>
struct SameMatrixMatcher : testing::MatcherInterface<Matrix const &>
{
    using Scalar = typename Matrix::Scalar;
    SameMatrixMatcher(Matrix const & expected, Scalar epsilon) :
        mExpected(expected),
        mEpsilon(epsilon)
    {}

    bool MatchAndExplain(Matrix const & matrix, testing::MatchResultListener* listener) const override
    {
        Matrix difference = mExpected - matrix;
        auto differenceMagnitude = difference.norm();
        *listener << "different by " << differenceMagnitude;
        return differenceMagnitude < mEpsilon;
    }

    void DescribeTo(std::ostream* os) const override
    {
        *os << "same as\n" << mExpected;
    }

    void DescribeNegationTo(std::ostream* os) const override
    {
        *os << "different from\n" << mExpected;
    }
private:
    Matrix const mExpected;
    Scalar const mEpsilon;
};

template <typename Quaternion>
struct SameOrientationMatcher : testing::MatcherInterface<Quaternion const &>
{
    using Scalar = typename Quaternion::Scalar;

    SameOrientationMatcher(Quaternion const & expected, Scalar epsilon) :
        mExpected(expected),
        mEpsilon(epsilon)
    {}

    bool MatchAndExplain(Quaternion const & quaternion, testing::MatchResultListener* listener) const override
    {
        auto differenceMagnitude = 1 - std::abs(mExpected.dot(quaternion));
        *listener << "different by " << differenceMagnitude;
        return differenceMagnitude < mEpsilon;
    }

    void DescribeTo(std::ostream* os) const override
    {
        *os << "same as [ " << mExpected.coeffs().transpose() << " ]";
    }

    void DescribeNegationTo(std::ostream* os) const override
    {
        *os << "different from [ " << mExpected.coeffs().transpose() << " ]";
    }
private:
    Quaternion const mExpected;
    Scalar const mEpsilon;
};

template<typename T>
inline auto IsSameMatrix(T const & expected, typename T::Scalar epsilon) {
    return testing::MakePolymorphicMatcher(SameMatrixMatcher<typename T::PlainMatrix>(expected, epsilon));
}

template<typename T>
inline auto IsSameOrientation(T const & expected, typename T::Scalar epsilon) {
    return testing::MakePolymorphicMatcher(SameOrientationMatcher<Eigen::Quaternion<typename T::Scalar>>(expected, epsilon));
}
