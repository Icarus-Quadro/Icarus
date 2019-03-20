#pragma once

#include <gmock/gmock-matchers.h>

#include <Eigen/Geometry>

namespace Eigen {
    template<typename T>
    inline std::ostream& operator<<(std::ostream& o, Eigen::Quaternion<T> const & q)
    {
        Eigen::IOFormat format(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        return o << q.coeffs().transpose().format(format);
    }
}

template <typename T>
struct SameOrientationMatcher
{
    SameOrientationMatcher(Eigen::Quaternion<T> const & expected, T epsilon) :
        mExpected(expected),
        mEpsilon(epsilon)
    {}

    bool MatchAndExplain(Eigen::Quaternion<T> const & q, testing::MatchResultListener* listener) const
    {
        // *listener << "actual: " << q;
        return std::abs(mExpected.dot(q)) > 1.0f - mEpsilon;
    }

    void DescribeTo(std::ostream* os) const
    {
        *os << "same as " << mExpected;
    }

    // Describes the property of a value NOT matching this matcher.
    void DescribeNegationTo(::std::ostream* os) const
    {
        *os << "different from " << mExpected;
    }
private:
    Eigen::Quaternion<T> const mExpected;
    T const mEpsilon;
};

template<typename T>
inline testing::PolymorphicMatcher<SameOrientationMatcher<T>> IsSameOrientationAs(Eigen::Quaternion<T> const & expected, T epsilon) {
    return testing::MakePolymorphicMatcher(SameOrientationMatcher(expected, epsilon));
}
