#pragma once

#include <Eigen/Dense>
#include <cstddef>

namespace icarus
{
    template<typename T, size_t N>
    struct VarianceEstimator
    {
        VarianceEstimator(size_t sampleSize) :
            mSampleSize(sampleSize),
            mSampleNumber(0)
        {
            mMean.setZero();
            mMoment.setZero();
        }

        void addSample(Eigen::Matrix<T, N, 1> const & sample)
        {
            ++mSampleNumber;
            auto oldMean = mMean;
            mMean += (sample - mMean) / T(mSampleNumber);
            mMoment += (sample - oldMean).cwiseProduct(sample - mMean);
        }

        Eigen::Matrix<T, N, 1> mean() const
        {
            return mMean;
        }

        Eigen::Matrix<T, N, 1> variance() const
        {
            return mMoment / mSampleSize;
        }
    private:
        size_t mSampleSize;
        size_t mSampleNumber;
        Eigen::Matrix<T, N, 1> mMean;
        Eigen::Matrix<T, N, 1> mMoment;
    };
}
