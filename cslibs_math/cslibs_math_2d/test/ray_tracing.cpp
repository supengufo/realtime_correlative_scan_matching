#include <gtest/gtest.h>

#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math_2d/algorithms/amanatides.hpp>
#include <cslibs_math_2d/algorithms/bresenham.hpp>
#include <cslibs_math/utility/tiny_time.hpp>

const std::size_t ITERATIONS = 1000000;

TEST( Test_cslibs_math_2d, testAmanatidesDryrun)
{
    auto test = [](const cslibs_math_2d::Point2d &p0,
                   const cslibs_math_2d::Point2d &p1)
    {
        cslibs_math_2d::algorithms::Amanatides<double> a0(p0, p1, 1.0);

        while(!a0.done()) {
            std::cout << "[" << a0.x() << "," << a0.y() << "]";
            ++a0;
        }
        std::cout << "[" << a0.x() << "," << a0.y() << "]" << std::endl;
    };
    test(cslibs_math_2d::Point2d(0.5,0.5),
         cslibs_math_2d::Point2d(11.5, 4.5));

    test(cslibs_math_2d::Point2d(-0.5,-0.5),
         cslibs_math_2d::Point2d(-11.5, -4.5));
}


TEST( Test_cslibs_math_2d, testAmanatidesRuntime)
{
    auto exec =  [](const cslibs_math_2d::Point2d &p0,
                    const cslibs_math_2d::Point2d &p1)
    {
        cslibs_math_2d::algorithms::Amanatides<double> a0(p0, p1, 1.0);
        while(!a0.done()) {
            ++a0;
        }
    };

    cslibs_math::utility::tiny_time::time_t start;
    cslibs_math::utility::tiny_time::duration_t dur;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const cslibs_math_2d::Point2d p0 = cslibs_math_2d::Point2d::random();
        const cslibs_math_2d::Point2d p1 = cslibs_math_2d::Point2d::random();

        start = cslibs_math::utility::tiny_time::clock_t::now();
        exec(p0, p1);
        dur += cslibs_math::utility::tiny_time::clock_t::now() - start;
    }
    std::cout << "[ runtime  ] " << cslibs_math::utility::tiny_time::milliseconds(dur) / ITERATIONS << "ms" << std::endl;
}

TEST( Test_cslibs_math_2d, testBresenhamDryrun)
{
    auto test = [](const cslibs_math_2d::Point2d &p0,
                    const cslibs_math_2d::Point2d &p1)
    {
        cslibs_math_2d::algorithms::Bresenham a0(p0, p1, 1.0);

        while(!a0.done()) {
            std::cout << "[" << a0.x() << "," << a0.y() << "]";
            ++a0;
        }
        std::cout << "[" << a0.x() << "," << a0.y() << "]" << std::endl;
    };

    test(cslibs_math_2d::Point2d(0.5,0.5),
         cslibs_math_2d::Point2d(11.5, 4.5));

    test(cslibs_math_2d::Point2d(-0.5,-0.5),
         cslibs_math_2d::Point2d(-11.5, -4.5));
}

TEST( Test_cslibs_math_2d, testBresenhamRuntime)
{
    auto exec =  [](const cslibs_math_2d::Point2d &p0,
                    const cslibs_math_2d::Point2d &p1)
    {
        cslibs_math_2d::algorithms::Amanatides<double> a0(p0, p1, 1.0);
        while(!a0.done()) {
            ++a0;
        }
    };

    cslibs_math::utility::tiny_time::time_t start;
    cslibs_math::utility::tiny_time::duration_t dur;
    for(std::size_t i = 0 ; i < ITERATIONS ; ++i) {
        const cslibs_math_2d::Point2d p0 = cslibs_math_2d::Point2d::random();
        const cslibs_math_2d::Point2d p1 = cslibs_math_2d::Point2d::random();

        start = cslibs_math::utility::tiny_time::clock_t::now();
        exec(p0, p1);
        dur += cslibs_math::utility::tiny_time::clock_t::now() - start;
    }
    std::cout << "[ runtime  ] " << cslibs_math::utility::tiny_time::milliseconds(dur) / ITERATIONS << "ms" << std::endl;
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
