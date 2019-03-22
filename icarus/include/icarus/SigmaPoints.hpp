#pragma once

#include <Eigen/Dense>

namespace icarus {
    template<typename T, size_t N, size_t M>
    struct SigmaPoints
    {
        std::array<Eigen::Matrix<T, N, 1>, M> points;
        std::array<T, M> meanWeights;
        std::array<T, M> covarianceWeights;

        static constexpr size_t size() { return M; }
    };
}