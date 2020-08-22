#include <gtest/gtest.h>

#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_2d/linear/point.hpp>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/common/angle.hpp>
#include <cslibs_math_2d/linear/vector.hpp>

#include <tf/tf.h>

using rng_t = cslibs_math::random::Uniform<double,1>;

const std::size_t REPETITIONS = 10000;

TEST(Test_cslibs_math_2d, testInitialization)
{
    rng_t rng(-10.0, 10.0);

    cslibs_math_2d::Vector2d v0;
    EXPECT_EQ(v0(0), 0.0);
    EXPECT_EQ(v0(1), 0.0);

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        const double val1 = rng.get();
        cslibs_math_2d::Vector2d v1(val1);
        EXPECT_EQ(v1(0), val1);
        EXPECT_EQ(v1(1), val1);

        const double val2 = rng.get();
        cslibs_math_2d::Vector2d v2(val1,val2);
        EXPECT_EQ(v2(0), val1);
        EXPECT_EQ(v2(1), val2);

        cslibs_math_2d::Vector2d v3(v2);
        EXPECT_EQ(v3(0), v2(0));
        EXPECT_EQ(v3(1), v2(1));
    }
}

TEST(Test_cslibs_math_2d, testArrayConversion)
{
    rng_t rng(-10.0, 10.0);

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        std::array<double, 2> a0 = {{rng.get(), rng.get()}};
        cslibs_math_2d::Vector2d v0(a0);
        EXPECT_EQ(v0(0), a0[0]);
        EXPECT_EQ(v0(1), a0[1]);

        cslibs_math_2d::Vector2d v1(rng.get(), rng.get());
        std::array<double, 2> a1 = v1.array();
        EXPECT_EQ(a1[0], v1(0));
        EXPECT_EQ(a1[1], v1(1));
    }
}

TEST(Test_cslibs_math_2d, testMul)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x = rng.get();
        const double y = rng.get();
        const double s = rng.get();
        cslibs_math_2d::Vector2d v0(x,y);
        cslibs_math_2d::Vector2d v1 = v0 * s;

        EXPECT_NEAR(v1(0), s * x, 1e-9);
        EXPECT_NEAR(v1(1), s * y, 1e-9);

        v0 *= s;
        EXPECT_NEAR(v0(0), s * x, 1e-9);
        EXPECT_NEAR(v0(1), s * y, 1e-9);
    }
}

TEST(Test_cslibs_math_2d, testDiv)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x = rng.get();
        const double y = rng.get();
        double s = rng.get();
        while(s == 0.0)
            s = rng.get();

        cslibs_math_2d::Vector2d v0(x,y);
        cslibs_math_2d::Vector2d v1 = v0 / s;

        EXPECT_NEAR(v1(0), x / s, 1e-9);
        EXPECT_NEAR(v1(1), y / s, 1e-9);

        v0 /= s;
        EXPECT_NEAR(v0(0), x / s, 1e-9);
        EXPECT_NEAR(v0(1), y / s, 1e-9);
    }
}

TEST(Test_cslibs_math_2d, testAdd)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d v2 = v0 + v1;
        EXPECT_EQ(v2(0), x0 + x1);
        EXPECT_EQ(v2(1), y0 + y1);

        cslibs_math_2d::Vector2d v3 = v1 + v0;
        EXPECT_EQ(v3(0), x0 + x1);
        EXPECT_EQ(v3(1), y0 + y1);

        v1 += v0;
        EXPECT_EQ(v1(0), x0 + x1);
        EXPECT_EQ(v1(1), y0 + y1);
    }
}

TEST(Test_cslibs_math_2d, testSub)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d v2 = v0 - v1;
        EXPECT_EQ(v2(0), x0 - x1);
        EXPECT_EQ(v2(1), y0 - y1);

        cslibs_math_2d::Vector2d v3 = v1 - v0;
        EXPECT_EQ(v3(0), x1 - x0);
        EXPECT_EQ(v3(1), y1 - y0);

        v1 -= v0;
        EXPECT_EQ(v1(0), x1 - x0);
        EXPECT_EQ(v1(1), y1 - y0);
    }
}

TEST(Test_cslibs_math_2d, testDot)
{
    rng_t rng(-10.0, 10.0);
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        EXPECT_NEAR(v0.dot(v1), x0 * x1 + y0 * y1, 1e-9);
        EXPECT_NEAR(v1.dot(v0), x1 * x0 + y1 * y0, 1e-9);
    }
}

TEST(Test_cslibs_math_2d, testLenAngNorm)
{
    rng_t rng_dir(-M_PI, M_PI);
    rng_t rng_len(0.0, 10.0);

    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        const double a = rng_dir.get();
        const double l = rng_len.get();

        cslibs_math_2d::Vector2d v0(std::cos(a) * l, std::sin(a) * l);
        EXPECT_NEAR(v0.length(),  std::abs(l), 1e-5);
        EXPECT_NEAR(v0.length2(), l*l, 1e-5);
        EXPECT_NEAR(cslibs_math_2d::angle(v0), a, 1e-5);



        cslibs_math_2d::Vector2d v1 = v0.normalized();
        EXPECT_NEAR(v1.length(), 1.0, 1e-5);
    }
}

TEST(Test_cslibs_math_2d, testAssign)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1;
        v1 = v0;

        EXPECT_EQ(v1(0), v0(0));
        EXPECT_EQ(v1(1), v1(1));
    }
}

TEST(Test_cslibs_math_2d, testMinMax)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);

        cslibs_math_2d::Vector2d min = cslibs_math::linear::min(v0,v1);
        cslibs_math_2d::Vector2d max = cslibs_math::linear::max(v0,v1);

        EXPECT_EQ(min(0), std::min(v0(0), v1(0)));
        EXPECT_EQ(min(1), std::min(v0(1), v1(1)));

        EXPECT_EQ(max(0), std::max(v0(0), v1(0)));
        EXPECT_EQ(max(1), std::max(v0(1), v1(1)));

        min = cslibs_math::linear::min(v1, v0);
        max = cslibs_math::linear::max(v1, v0);

        EXPECT_EQ(min(0), std::min(v0(0), v1(0)));
        EXPECT_EQ(min(1), std::min(v0(1), v1(1)));

        EXPECT_EQ(max(0), std::max(v0(0), v1(0)));
        EXPECT_EQ(max(1), std::max(v0(1), v1(1)));
    }
}


TEST(Test_cslibs_math_2d, testDistance)
{
    for(std::size_t i = 0 ; i < REPETITIONS ; ++i) {
        rng_t rng(-10.0, 10.0);

        const double x0 = rng.get();
        const double y0 = rng.get();
        const double x1 = rng.get();
        const double y1 = rng.get();

        cslibs_math_2d::Vector2d v0(x0,y0);
        cslibs_math_2d::Vector2d v1(x1,y1);
        cslibs_math_2d::Vector2d d = v0 - v1;

        EXPECT_NEAR(cslibs_math::linear::distance(v0, v1), d.length(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance(v1, v0), d.length(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance2(v0, v1), d.length2(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance2(v1, v0), d.length2(), 1e-6);


        d = v1 - v0;
        EXPECT_NEAR(cslibs_math::linear::distance(v0, v1), d.length(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance(v1, v0), d.length(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance2(v0, v1), d.length2(), 1e-6);
        EXPECT_NEAR(cslibs_math::linear::distance2(v1, v0), d.length2(), 1e-6);

    }
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
