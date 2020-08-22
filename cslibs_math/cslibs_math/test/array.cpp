#include <gtest/gtest.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/common/array.hpp>

#include <cslibs_math/linear/vector.hpp>
#include <cslibs_math/linear/matrix.hpp>

using rng_t = cslibs_math::random::Uniform<double,1>;
const std::size_t REPETITIONS = 10000;

#include <eigen3/Eigen/Core>

TEST(Test_cslibs_math, testArrayMinus)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_2 = i_2_1 - i_2_0;
        EXPECT_EQ(i_2_2[0], i_2_1[0] - i_2_0[0]);
        EXPECT_EQ(i_2_2[1], i_2_1[1] - i_2_0[1]);

        std::array<double, 2> d_2_0 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_2 = d_2_1 - d_2_0;
        EXPECT_EQ(d_2_2[0], d_2_1[0] - d_2_0[0]);
        EXPECT_EQ(d_2_2[1], d_2_1[1] - d_2_0[1]);

        std::array<int, 3> i_3_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 3> i_3_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};;
        std::array<int, 3> i_3_2  = i_3_1 - i_3_0;
        EXPECT_EQ(i_3_2[0], i_3_1[0] - i_3_0[0]);
        EXPECT_EQ(i_3_2[1], i_3_1[1] - i_3_0[1]);
        EXPECT_EQ(i_3_2[2], i_3_1[2] - i_3_0[2]);

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_2 = d_3_1 - d_3_0;
        EXPECT_EQ(d_3_2[0], d_3_1[0] - d_3_0[0]);
        EXPECT_EQ(d_3_2[1], d_3_1[1] - d_3_0[1]);
        EXPECT_EQ(d_3_2[2], d_3_1[2] - d_3_0[2]);
    }
}


TEST(Test_cslibs_math, testArrayPlus)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{static_cast<int>(rng.get()),
                                     static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_1 = {{static_cast<int>(rng.get()),
                                     static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_2 = i_2_1 + i_2_0;
        EXPECT_EQ(i_2_2[0], i_2_1[0] + i_2_0[0]);
        EXPECT_EQ(i_2_2[1], i_2_1[1] + i_2_0[1]);

        std::array<double, 2> d_2_0 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_2 = d_2_1 + d_2_0;
        EXPECT_EQ(d_2_2[0], d_2_1[0] + d_2_0[0]);
        EXPECT_EQ(d_2_2[1], d_2_1[1] + d_2_0[1]);

        std::array<int, 3> i_3_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 3> i_3_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};;
        std::array<int, 3> i_3_2  = i_3_1 + i_3_0;
        EXPECT_EQ(i_3_2[0], i_3_1[0] + i_3_0[0]);
        EXPECT_EQ(i_3_2[1], i_3_1[1] + i_3_0[1]);
        EXPECT_EQ(i_3_2[2], i_3_1[2] + i_3_0[2]);

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_2 = d_3_1 + d_3_0;
        EXPECT_EQ(d_3_2[0], d_3_1[0] + d_3_0[0]);
        EXPECT_EQ(d_3_2[1], d_3_1[1] + d_3_0[1]);
        EXPECT_EQ(d_3_2[2], d_3_1[2] + d_3_0[2]);
    }
}

TEST(Test_cslibs_math, testArrayMult)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_2 = i_2_1 * i_2_0;
        EXPECT_EQ(i_2_2[0], i_2_1[0] * i_2_0[0]);
        EXPECT_EQ(i_2_2[1], i_2_1[1] * i_2_0[1]);
        const int mi = rng.get();
        i_2_2 = i_2_2 * mi;
        EXPECT_EQ(i_2_2[0], i_2_1[0] * i_2_0[0] * mi);
        EXPECT_EQ(i_2_2[1], i_2_1[1] * i_2_0[1] * mi);

        std::array<double, 2> d_2_0 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_2 = d_2_1 * d_2_0;
        EXPECT_EQ(d_2_2[0], d_2_1[0] * d_2_0[0]);
        EXPECT_EQ(d_2_2[1], d_2_1[1] * d_2_0[1]);
        const double md = rng.get();
        d_2_2 = d_2_2 * md;
        EXPECT_EQ(d_2_2[0], d_2_1[0] * d_2_0[0] * md);
        EXPECT_EQ(d_2_2[1], d_2_1[1] * d_2_0[1] * md);

        std::array<int, 3> i_3_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 3> i_3_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};;
        std::array<int, 3> i_3_2  = i_3_1 * i_3_0;
        EXPECT_EQ(i_3_2[0], i_3_1[0] * i_3_0[0]);
        EXPECT_EQ(i_3_2[1], i_3_1[1] * i_3_0[1]);
        EXPECT_EQ(i_3_2[2], i_3_1[2] * i_3_0[2]);
        i_3_2 = i_3_2 * mi;
        EXPECT_EQ(i_3_2[0], i_3_1[0] * i_3_0[0] * mi);
        EXPECT_EQ(i_3_2[1], i_3_1[1] * i_3_0[1] * mi);
        EXPECT_EQ(i_3_2[2], i_3_1[2] * i_3_0[2] * mi);

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {rng.get(), rng.get(), rng.get()};
        std::array<double, 3> d_3_2 = d_3_1 * d_3_0;
        EXPECT_EQ(d_3_2[0], d_3_1[0] * d_3_0[0]);
        EXPECT_EQ(d_3_2[1], d_3_1[1] * d_3_0[1]);
        EXPECT_EQ(d_3_2[2], d_3_1[2] * d_3_0[2]);
        d_3_2 = d_3_2 * md;
        EXPECT_EQ(d_3_2[0], d_3_1[0] * d_3_0[0] * md);
        EXPECT_EQ(d_3_2[1], d_3_1[1] * d_3_0[1] * md);
        EXPECT_EQ(d_3_2[2], d_3_1[2] * d_3_0[2] * md);
    }
}

TEST(Test_cslibs_math, testArrayDiv)
{
    rng_t rng(-100.0, 100.0);
    auto irand = [&rng]() {
        int r = 0;
        while(r == 0) {
            r = static_cast<int>(std::floor(0.5 + rng.get()));
        }
        return r;
    };

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{irand(),irand()}};
        std::array<int, 2> i_2_1 = {{irand(),irand()}};
        std::array<int, 2> i_2_2 = i_2_1  / i_2_0;
        EXPECT_NEAR(i_2_2[0], i_2_1[0]  / i_2_0[0], 1e-9);
        EXPECT_NEAR(i_2_2[1], i_2_1[1]  / i_2_0[1], 1e-9);
        const int mi = irand();
        i_2_2 = i_2_2 / mi;
        EXPECT_NEAR(i_2_2[0], i_2_1[0]  / i_2_0[0]  / mi, 1e-9);
        EXPECT_NEAR(i_2_2[1], i_2_1[1]  / i_2_0[1]  / mi, 1e-9);

        std::array<double, 2> d_2_0 = {{rng.get(), rng.getNEQ(0.0)}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.getNEQ(0.0)}};
        std::array<double, 2> d_2_2 = d_2_1  / d_2_0;
        EXPECT_NEAR(d_2_2[0], d_2_1[0]  / d_2_0[0], 1e-9);
        EXPECT_NEAR(d_2_2[1], d_2_1[1]  / d_2_0[1], 1e-9);
        const double md = rng.getNEQ(0.0);
        d_2_2 = d_2_2 / md;
        EXPECT_NEAR(d_2_2[0], d_2_1[0]  / d_2_0[0]  / md, 1e-9);
        EXPECT_NEAR(d_2_2[1], d_2_1[1]  / d_2_0[1]  / md, 1e-9);

        std::array<int, 3> i_3_0 = {{irand(), irand(), irand()}};
        std::array<int, 3> i_3_1 = {{irand(), irand(), irand()}};
        std::array<int, 3> i_3_2  = i_3_1  / i_3_0;
        EXPECT_NEAR(i_3_2[0], i_3_1[0]  / i_3_0[0], 1e-9);
        EXPECT_NEAR(i_3_2[1], i_3_1[1]  / i_3_0[1], 1e-9);
        EXPECT_NEAR(i_3_2[2], i_3_1[2]  / i_3_0[2], 1e-9);
        i_3_2 = i_3_2 / mi;
        EXPECT_NEAR(i_3_2[0], i_3_1[0]  / i_3_0[0]  / mi, 1e-9);
        EXPECT_NEAR(i_3_2[1], i_3_1[1]  / i_3_0[1]  / mi, 1e-9);
        EXPECT_NEAR(i_3_2[2], i_3_1[2]  / i_3_0[2]  / mi, 1e-9);

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_2 = d_3_1  / d_3_0;
        EXPECT_NEAR(d_3_2[0], d_3_1[0]  / d_3_0[0], 1e-9);
        EXPECT_NEAR(d_3_2[1], d_3_1[1]  / d_3_0[1], 1e-9);
        EXPECT_NEAR(d_3_2[2], d_3_1[2]  / d_3_0[2], 1e-9);
        d_3_2 = d_3_2 / md;
        EXPECT_NEAR(d_3_2[0], d_3_1[0]  / d_3_0[0]  / md, 1e-9);
        EXPECT_NEAR(d_3_2[1], d_3_1[1]  / d_3_0[1]  / md, 1e-9);
        EXPECT_NEAR(d_3_2[2], d_3_1[2]  / d_3_0[2]  / md, 1e-9);
    }
}

TEST(Test_cslibs_math, testArrayMin)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};

        std::array<int, 2> i_2_2 = std::min(i_2_0, i_2_1);
        EXPECT_EQ(i_2_2[0], std::min(i_2_1[0], i_2_0[0]));
        EXPECT_EQ(i_2_2[1], std::min(i_2_1[1], i_2_0[1]));
        i_2_2 = std::min(i_2_1, i_2_0);
        EXPECT_EQ(i_2_2[0], std::min(i_2_1[0], i_2_0[0]));
        EXPECT_EQ(i_2_2[1], std::min(i_2_1[1], i_2_0[1]));


        std::array<double, 2> d_2_0 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.get()}};

        std::array<double, 2> d_2_2 = std::min(d_2_0, d_2_1);
        EXPECT_EQ(d_2_2[0], std::min(d_2_1[0], d_2_0[0]));
        EXPECT_EQ(d_2_2[1], std::min(d_2_1[1], d_2_0[1]));
        d_2_2 = std::min(d_2_1, d_2_0);
        EXPECT_EQ(d_2_2[0], std::min(d_2_0[0], d_2_1[0]));
        EXPECT_EQ(d_2_2[1], std::min(d_2_0[1], d_2_1[1]));

        std::array<int, 3> i_3_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 3> i_3_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};

        std::array<int, 3> i_3_2  = std::min(i_3_1, i_3_0);
        EXPECT_EQ(i_3_2[0], std::min(i_3_1[0], i_3_0[0]));
        EXPECT_EQ(i_3_2[1], std::min(i_3_1[1], i_3_0[1]));
        EXPECT_EQ(i_3_2[2], std::min(i_3_1[2], i_3_0[2]));
        i_3_2  = std::min(i_3_0, i_3_1);
        EXPECT_EQ(i_3_2[0], std::min(i_3_1[0], i_3_0[0]));
        EXPECT_EQ(i_3_2[1], std::min(i_3_1[1], i_3_0[1]));
        EXPECT_EQ(i_3_2[2], std::min(i_3_1[2], i_3_0[2]));

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {{rng.get(), rng.get(), rng.get()}};

        std::array<double, 3> d_3_2 = std::min(d_3_1, d_3_0);
        EXPECT_EQ(d_3_2[0], std::min(d_3_1[0], d_3_0[0]));
        EXPECT_EQ(d_3_2[1], std::min(d_3_1[1], d_3_0[1]));
        EXPECT_EQ(d_3_2[2], std::min(d_3_1[2], d_3_0[2]));
        d_3_2 = std::min(d_3_0, d_3_1);
        EXPECT_EQ(d_3_2[0], std::min(d_3_1[0], d_3_0[0]));
        EXPECT_EQ(d_3_2[1], std::min(d_3_1[1], d_3_0[1]));
        EXPECT_EQ(d_3_2[2], std::min(d_3_1[2], d_3_0[2]));
    }
}

TEST(Test_cslibs_math, testArrayMax)
{
    rng_t rng(-100.0, 100.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<int, 2> i_2_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 2> i_2_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};

        std::array<int, 2> i_2_2 = std::max(i_2_0, i_2_1);
        EXPECT_EQ(i_2_2[0], std:: max(i_2_1[0], i_2_0[0]));
        EXPECT_EQ(i_2_2[1], std:: max(i_2_1[1], i_2_0[1]));
        i_2_2 = std::max(i_2_1, i_2_0);
        EXPECT_EQ(i_2_2[0], std:: max(i_2_1[0], i_2_0[0]));
        EXPECT_EQ(i_2_2[1], std:: max(i_2_1[1], i_2_0[1]));


        std::array<double, 2> d_2_0 = {{rng.get(), rng.get()}};
        std::array<double, 2> d_2_1 = {{rng.get(), rng.get()}};

        std::array<double, 2> d_2_2 = std::max(d_2_0, d_2_1);
        EXPECT_EQ(d_2_2[0], std:: max(d_2_1[0], d_2_0[0]));
        EXPECT_EQ(d_2_2[1], std:: max(d_2_1[1], d_2_0[1]));
        d_2_2 = std::max(d_2_1, d_2_0);
        EXPECT_EQ(d_2_2[0], std:: max(d_2_0[0], d_2_1[0]));
        EXPECT_EQ(d_2_2[1], std:: max(d_2_0[1], d_2_1[1]));

        std::array<int, 3> i_3_0 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};
        std::array<int, 3> i_3_1 = {{static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get()),
                                    static_cast<int>(rng.get())}};

        std::array<int, 3> i_3_2  = std::max(i_3_1, i_3_0);
        EXPECT_EQ(i_3_2[0], std:: max(i_3_1[0], i_3_0[0]));
        EXPECT_EQ(i_3_2[1], std:: max(i_3_1[1], i_3_0[1]));
        EXPECT_EQ(i_3_2[2], std:: max(i_3_1[2], i_3_0[2]));
        i_3_2  = std::max(i_3_0, i_3_1);
        EXPECT_EQ(i_3_2[0], std:: max(i_3_1[0], i_3_0[0]));
        EXPECT_EQ(i_3_2[1], std:: max(i_3_1[1], i_3_0[1]));
        EXPECT_EQ(i_3_2[2], std:: max(i_3_1[2], i_3_0[2]));

        std::array<double, 3> d_3_0 = {{rng.get(), rng.get(), rng.get()}};
        std::array<double, 3> d_3_1 = {{rng.get(), rng.get(), rng.get()}};

        std::array<double, 3> d_3_2 = std::max(d_3_1, d_3_0);
        EXPECT_EQ(d_3_2[0], std:: max(d_3_1[0], d_3_0[0]));
        EXPECT_EQ(d_3_2[1], std:: max(d_3_1[1], d_3_0[1]));
        EXPECT_EQ(d_3_2[2], std:: max(d_3_1[2], d_3_0[2]));
        d_3_2 = std::max(d_3_0, d_3_1);
        EXPECT_EQ(d_3_2[0], std:: max(d_3_1[0], d_3_0[0]));
        EXPECT_EQ(d_3_2[1], std:: max(d_3_1[1], d_3_0[1]));
        EXPECT_EQ(d_3_2[2], std:: max(d_3_1[2], d_3_0[2]));
    }
}

TEST(Test_cslibs_math, testArrayAligned)
{
    using vector_t = cslibs_math::linear::Vector<double, 3>;
    using matrix_t = cslibs_math::linear::Matrix<double, 3, 3>;

    using vector_array_t = std::array<vector_t, 3>;
    using matrix_array_t = std::array<matrix_t, 3>;

    vector_t v1 = vector_t::random();
    vector_t v2 = vector_t::random();
    vector_t v3 = vector_t::random();

    matrix_t m1 = matrix_t::random();
    matrix_t m2 = matrix_t::random();
    matrix_t m3 = matrix_t::random();

    /// test init
    vector_array_t va1 = {{v1, v2, v3}};
    matrix_array_t ma1 = {{m1, m2, m3}};

    EXPECT_EQ(v1.data(), va1[0].data());
    EXPECT_EQ(v2.data(), va1[1].data());
    EXPECT_EQ(v3.data(), va1[2].data());

    EXPECT_EQ(m1.data(), ma1[0].data());
    EXPECT_EQ(m2.data(), ma1[1].data());
    EXPECT_EQ(m3.data(), ma1[2].data());

    /// test assign
    v1 = vector_t::random();
    v2 = vector_t::random();
    v3 = vector_t::random();

    m1 = matrix_t::random();
    m2 = matrix_t::random();
    m3 = matrix_t::random();

    va1[0]  = v1;
    va1[1]  = v2;
    va1[2]  = v3;
    ma1[0]  = m1;
    ma1[1]  = m2;
    ma1[2]  = m3;

    EXPECT_EQ(v1.data(), va1[0].data());
    EXPECT_EQ(v2.data(), va1[1].data());
    EXPECT_EQ(v3.data(), va1[2].data());

    EXPECT_EQ(m1.data(), ma1[0].data());
    EXPECT_EQ(m2.data(), ma1[1].data());
    EXPECT_EQ(m3.data(), ma1[2].data());
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
