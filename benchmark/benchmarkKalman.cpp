#include <benchmark/benchmark.h>

#include <icarus/UnscentedKalmanFilter.hpp>
#include <icarus/GyroscopeMeasurementModel.hpp>
#include <icarus/RigidBodyProcessModel.hpp>

static void benchmarkKalman(benchmark::State& benchmark)
{
    icarus::UnscentedKalmanFilter<float, 7> kalman;
    auto & s = kalman.state<icarus::RigidBodyProcessModel<float>::State &>();

    icarus::RigidBodyProcessModel<float> processModel;
    icarus::GyroscopeMeasurementModel<float> measurementModel;

    icarus::GaussianDistribution<float, 3> measurement;
    measurement.mean << 3.1415f, 0.0f, 0.0f;
    measurement.covariance = Eigen::Matrix<float, 3, 3>::Identity() * 0.001f;

    auto iterations = benchmark.range(0);

    for (auto _ : benchmark) {
        s.orientation.setIdentity();
        s.angularMomentum.setZero();

        for (int i = 0; i < iterations; ++i) {
            kalman.filter(processModel, measurementModel, measurement, 0.01f);
            s.orientation.normalize();
        }
    }
}

BENCHMARK(benchmarkKalman)->Range(16, 1 << 16);

template<size_t N>
void benchmarkKalmanComplexity(benchmark::State& benchmark)
{
    struct {
        Eigen::Matrix<float, N, 1> operator()(Eigen::Matrix<float, N, 1> const & state, float timeStep) const
        {
            Eigen::Matrix<float, N, 1> ret;
            ret = state;
            ret.template tail<N - 1>() += ret.template head<N - 1>();
            ret[0] += 1.0f;
            return ret;
        }
        Eigen::Matrix<float, N, N> noise() const
        {
            Eigen::Matrix<float, N, N> ret;
            ret.setIdentity();
            ret *= 0.001f;
            return ret;
        }
    } processModel;

    struct {
        Eigen::Matrix<float, N / 2, 1> operator()(Eigen::Matrix<float, N, 1> const & state) const
        {
            Eigen::Matrix<float, N / 2, 1> ret;
            ret = state.template head<N / 2>();
            return ret;
        }
    } measurementModel;

    icarus::UnscentedKalmanFilter<float, N> kalman;

    icarus::GaussianDistribution<float, N / 2> measurement;
    measurement.mean.setRandom();
    measurement.covariance.setIdentity();

    for (auto _ : benchmark) {
        kalman.stateVector().setRandom();

        for (int i = 0; i < 100; ++i) {
            kalman.filter(processModel, measurementModel, measurement, 0.01f);
        }
    }
}

static void benchmarkKalmanComplexity4(benchmark::State& benchmark)
{
    benchmarkKalmanComplexity<4>(benchmark);
}

BENCHMARK(benchmarkKalmanComplexity4);

static void benchmarkKalmanComplexity8(benchmark::State& benchmark)
{
    benchmarkKalmanComplexity<8>(benchmark);
}

BENCHMARK(benchmarkKalmanComplexity8);

static void benchmarkKalmanComplexity16(benchmark::State& benchmark)
{
    benchmarkKalmanComplexity<16>(benchmark);
}

BENCHMARK(benchmarkKalmanComplexity16);