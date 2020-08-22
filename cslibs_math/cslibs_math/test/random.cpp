#include <gtest/gtest.h>

#include <cslibs_math/random/random.hpp>

TEST(Test_cslibs_math, testNorma1D)
{
    const std::size_t size = 100000;

    cslibs_math::random::Normal<double,1> rng(0.0, 1.0);
    std::vector<double> seq_1(size, 0.0);
    std::vector<double> seq_2(size, 0.0);
    std::vector<double> seq_3(size, 0.0);
    std::vector<double> seq_4(size, 0.0);

    for(std::size_t i = 0 ; i < size ; ++i)
        seq_1[i] = rng.get();

    rng.set(0.0, 1.0);
    for(std::size_t i = 0 ; i < size ; ++i)
        seq_2[i] = rng.get();

    rng.set(0.0, 2.0);
    for(std::size_t i = 0 ; i < size ; ++i)
        seq_3[i] = rng.get();

    rng.set(0.0, 1.0);
    for(std::size_t i = 0 ; i < size ; ++i)
        seq_4[i] = rng.get();


    for(std::size_t i = 0 ; i < size ; ++i)
        EXPECT_NE(seq_1[i], seq_2[i]);

    for(std::size_t i = 0 ; i < size ; ++i)
        EXPECT_NE(seq_1[i], seq_4[i]);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
