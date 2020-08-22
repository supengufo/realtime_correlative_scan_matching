#include <gtest/gtest.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/common/sqrt.hpp>

using rng_t = cslibs_math::random::Uniform<double,1>;
const std::size_t REPETITIONS = 10000;

TEST(Test_cslibs_math, testSqrt)
{
    EXPECT_NEAR(cslibs_math::common::sqrt(M_PI), std::sqrt(M_PI), 1e-9);

    rng_t rng(0.0, 10000000.0);
    for (std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        const double val = rng.get();
        EXPECT_NEAR(cslibs_math::common::sqrt(val), std::sqrt(val), 1e-9);
    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
