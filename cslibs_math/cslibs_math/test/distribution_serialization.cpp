#include <gtest/gtest.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/serialization/distribution.hpp>

const std::size_t MIN_DIMENSION   = 1;
const std::size_t MAX_DIMENSION   = 3;
const std::size_t REPETITIONS     = 1000;
const std::size_t MIN_NUM_SAMPLES = 100;
const std::size_t MAX_NUM_SAMPLES = 1000;

template <std::size_t Dim>
using rng_t = typename cslibs_math::random::Uniform<double,Dim>;

template <std::size_t minDim, std::size_t Dim>
struct Loop {
    template<template <std::size_t> class Function>
    static void iterate() {
        Function<Dim>()();
        Loop<minDim, Dim-1>::template iterate<Function>();
    }
};

template <std::size_t Dim>
struct Loop<Dim, Dim> {
    template<template <std::size_t> class Function>
    static void iterate() {
        Function<Dim>()();
    }
};

template <std::size_t Dim>
struct TestDimension {
    void operator()() {
        if (Dim < 1)
            return;

        Eigen::Matrix<double, Dim, 1> min_sample, max_sample;
        for (std::size_t i = 0 ; i < Dim ; ++ i) {
            min_sample[i] = -100.0;
            max_sample[i] =  100.0;
        }

        rng_t<1>   rng_num_samples(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
        rng_t<Dim> rng(min_sample, max_sample);

        using distribution_t = cslibs_math::statistics::Distribution<double,Dim, 3>;
        for (std::size_t i = 0 ; i < REPETITIONS ; ++ i) {
             distribution_t d;

            for (std::size_t n = 0 ; n < static_cast<std::size_t>(rng_num_samples.get()) ; ++ n)
                d.add(rng.get());

            // serialization
            YAML::Node n(d);

            // de-serialization
            const distribution_t & d_converted = n.as<distribution_t>();

            // tests
            EXPECT_EQ(d.getN(), d_converted.getN());
            for (std::size_t i = 0 ; i < Dim ; ++ i) {
                EXPECT_NEAR(d.getMean()(i), d_converted.getMean()(i), 1e-9);
                for (std::size_t j = 0 ; j < Dim ; ++ j) {
                    EXPECT_NEAR(d.getCorrelated()(i, j),        d_converted.getCorrelated()(i, j),        1e-9);
                    EXPECT_NEAR(d.getCovariance()(i, j),        d_converted.getCovariance()(i, j),        1e-9);
                    EXPECT_NEAR(d.getInformationMatrix()(i, j), d_converted.getInformationMatrix()(i, j), 1e-9);
                }
            }
            EXPECT_NEAR(d.denominator(), d_converted.denominator(), 1e-9);
        }
    }
};

template <>
struct TestDimension<1> {
    void operator()() {
        rng_t<1> rng_num_samples(MIN_NUM_SAMPLES, MAX_NUM_SAMPLES);
        rng_t<1> rng(-100.0, 100.0);

        using distribution_t = cslibs_math::statistics::Distribution<double,1, 3>;
        for (std::size_t i = 0 ; i < REPETITIONS ; ++ i) {
             distribution_t d;

            for (std::size_t n = 0 ; n < static_cast<std::size_t>(rng_num_samples.get()) ; ++ n)
                d.add(rng.get());

            // serialization
            YAML::Node n(d);

            // de-serialization
            distribution_t d_converted = n.as<distribution_t>();

            // tests
            EXPECT_EQ(d.getN(),                   d_converted.getN());
            EXPECT_NEAR(d.getMean(),              d_converted.getMean(),              1e-9);
            EXPECT_NEAR(d.getVariance(),          d_converted.getVariance(),          1e-9);
            EXPECT_NEAR(d.getStandardDeviation(), d_converted.getStandardDeviation(), 1e-9);
        }
    }
};

TEST(Test_cslibs_math, testDistributionSerialization)
{
    // random tests over all dimensions
    Loop<MIN_DIMENSION, MAX_DIMENSION>::iterate<TestDimension>();
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
