#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math_2d/linear/ellipse.hpp>
#include <gtest/gtest.h>
#include <random>

using namespace cslibs_math_2d;

typedef Ellipse<double> EllipseD;
typedef EllipseFit<double> EllipseFitD;
TEST(Test_cslibs_math_2d, randomFullEllipseFit)
{
    std::uniform_real_distribution<double> unif(0,1.0);
    std::uniform_real_distribution<double> un_angle(0,M_PI - 0.01);
    std::size_t succ_tests = 0;
    std::size_t ntests = 100;
    std::default_random_engine re;
    for(std::size_t n = 0; n < ntests; ++n){
        double a  = unif(re);//0.2;
        double b  = unif(re);//0.3;
        double cx = unif(re);//0.15;
        double cy = unif(re);//0.2;
        double k  = un_angle(re);
        EllipseD e(a,b,cx,cy,k);

        double dp = 2*M_PI/20;
        std::vector<Vector2d, Vector2d::allocator_t> points;
        for(std::size_t i = 0; i < 20; ++i){
            Vector2d p = e.getPoint(i*dp);
            points.emplace_back(p);
        }

        EllipseFitD fit;
        if(!fit.fit(points))
            EXPECT_TRUE(false);

        if( e.equals(fit.solution)){
            ++succ_tests;
        } else {
            std::cout<< " failed: "
                     << a  << ", "
                     << b  << ", "
                     << cx << ", "
                     << cy << ", "
                     << k  << std::endl;
            std::cout<< " fitted: "
                     << fit.solution.axis(0)  << ", "
                     << fit.solution.axis(1) << ", "
                     << fit.solution.center(0) << ", "
                     << fit.solution.center(1) << ", "
                     << fit.solution.alpha  << std::endl;
        }

    }
    double percentage = ((double) succ_tests)/((double) ntests);
    std::cout << "Number of fits with < 1 % difference: " << succ_tests << " of " << ntests << ". In %: " << percentage *100<< std::endl;
    EXPECT_TRUE(percentage > 0.99);
}

TEST(Test_cslibs_math_2d, fixedEllipseFit)
{
    double a = 0.2;
    double b = 0.3;
    double cx = 0.15;
    double cy = 0.2;
    double k = std::sin(M_PI_4);
    EllipseD e(a,b,cx,cy,k);
    double dp = 2*M_PI/20;
    std::vector<Vector2d, Vector2d::allocator_t> points;
    for(std::size_t i = 0; i < 20; ++i){
        Vector2d p = e.getPoint(i*dp);
        points.emplace_back(p);
    }

    EllipseFitD fit;
    try{
        fit.fit(points);
    } catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
    }
    EllipseD ef = fit.solution;
    EXPECT_NEAR(ef.axis(0), a,1e-5);
    EXPECT_NEAR(ef.axis(1), b,1e-5);
    EXPECT_NEAR(ef.center(0), cx,1e-5);
    EXPECT_NEAR(ef.center(1), cy,1e-5);
    EXPECT_NEAR(ef.alpha, k,1e-5);
}

TEST(Test_cslibs_math_2d, fixedHalfEllipseFit)
{
    double a = 0.2;
    double b = 0.3;
    double cx = 0.15;
    double cy = 0.2;
    double k = std::sin(M_PI_4);
    EllipseD e(a,b,cx,cy,k);
    std::size_t steps = 6;
    double dp = M_PI/steps;
    std::vector<Vector2d, Vector2d::allocator_t> points;
    for(std::size_t i = 0; i < steps; ++i){
        Vector2d p = e.getPoint(i*dp);
        points.emplace_back(p);
    }
    EllipseFitD fit;
    try{
        fit.fit(points);
    } catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
    }
    EllipseD ef = fit.solution;

    EXPECT_NEAR(ef.axis(0), a,1e-5);
    EXPECT_NEAR(ef.axis(1), b,1e-5);
    EXPECT_NEAR(ef.center(0), cx,1e-5);
    EXPECT_NEAR(ef.center(1), cy,1e-5);
    EXPECT_NEAR(ef.alpha, k,1e-5);
}


TEST(Test_cslibs_math_2d, fixedthreeEightsEllipseFit)
{
    double a = 0.2;
    double b = 0.3;
    double cx = 0.15;
    double cy = 0.2;
    double k = std::sin(M_PI_4);
    EllipseD e(a,b,cx,cy,k);
    std::size_t steps = 6;
    double dp = 0.75*M_PI/steps;
    std::vector<Vector2d, Vector2d::allocator_t> points;
    for(std::size_t i = 0; i < steps; ++i){
        Vector2d p = e.getPoint(i*dp);
        points.emplace_back(p);
    }
    EllipseFitD fit;
    try{
        fit.fit(points);
    } catch(const std::exception& e){
        std::cerr << e.what() << std::endl;
    }
    EllipseD ef = fit.solution;

    EXPECT_NEAR(ef.axis(0), a,1e-5);
    EXPECT_NEAR(ef.axis(1), b,1e-5);
    EXPECT_NEAR(ef.center(0), cx,1e-5);
    EXPECT_NEAR(ef.center(1), cy,1e-5);
    EXPECT_NEAR(ef.alpha, k,1e-5);
}

TEST(Test_cslibs_math_2d, randomthreeEightsEllipseFit)
{
    std::uniform_real_distribution<double> unif(0,1.0);
    std::uniform_real_distribution<double> un_angle(0,M_PI - 0.01);
    std::size_t succ_tests = 0;
    std::size_t ntests = 100;
    std::default_random_engine re;
    for(std::size_t n = 0; n < ntests; ++n){
        double a = unif(re);//0.2;
        double b = unif(re);//0.3;
        double cx = unif(re);//0.15;
        double cy = unif(re);//0.2;
        double k = un_angle(re);
        EllipseD e(a,b,cx,cy,k);
        std::size_t steps = 6;
        double dp = 0.75*M_PI/steps;
        std::vector<Vector2d, Vector2d::allocator_t> points;
        for(std::size_t i = 0; i < steps; ++i){
            Vector2d p = e.getPoint(i*dp);
            points.emplace_back(p);
        }

        EllipseFitD fit;
        try{
            fit.fit(points);
        } catch(const std::exception& e){
            std::cerr << e.what() << std::endl;
        }
        EllipseD ef1 = fit.solution;
        EllipseD ef2 = fit.solution;
        EllipseD ef3 = fit.solution;
        EllipseD ef4 = fit.solution;

        bool test11 = e.equals(ef1);
        bool test12 = e.equals(ef2);
        bool test13 = e.equals(ef3);
        bool test14 = e.equals(ef4);

        if(test11 || test12 || test13 || test14){
            ++succ_tests;
        } else {
            std::cout<< " failed: "
                     << a  << ", "
                     << b  << ", "
                     << cx << ", "
                     << cy << ", "
                     << k  << std::endl;
            std::cout<< " fitted: "
                     << ef1.axis(0)  << ", "
                     << ef1.axis(1) << ", "
                     << ef1.center(0) << ", "
                     << ef1.center(1) << ", "
                     << ef1.alpha  << std::endl;
            std::cout<< " fitted: "
                     << ef2.axis(0)  << ", "
                     << ef2.axis(1) << ", "
                     << ef2.center(0) << ", "
                     << ef2.center(1) << ", "
                     << ef2.alpha  << std::endl << std::endl;
        }

    }
    double percentage = ((double) succ_tests)/((double) ntests);
    std::cout << "Number of fits with < 1 % difference: " << succ_tests << " of " << ntests << ". In %: " << percentage *100<< std::endl;
    EXPECT_TRUE(percentage > 0.99);
}



int main(int argc, char *argv[])
{



    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
