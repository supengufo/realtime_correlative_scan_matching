#include <cslibs_math/statistics/stable_distribution.hpp>
#include <cslibs_math/statistics/weighted_distribution.hpp>
#include <cslibs_math/statistics/stable_weighted_distribution.hpp>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "test_distribution.hpp"

cslibs_math::TestDistribution<2> test_distribution_200;
cslibs_math::TestDistribution<2> test_distribution_500;
cslibs_math::TestDistribution<2> test_distribution_5000;

namespace cs = cslibs_math::statistics;

TEST(Test_cslibs_math, testDistributionInsertion)
{
  cs::StableDistribution<float,2> distribution;
  EXPECT_EQ(0ul, distribution.getN());
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
    EXPECT_EQ(i, distribution.getN());
    Eigen::Matrix<float,2,1> data;
    data[0] = static_cast<float>(test_distribution_200.data[i][0]);
    data[1] = static_cast<float>(test_distribution_200.data[i][1]);
    distribution.add(data);
  }
  EXPECT_EQ(test_distribution_200.data.size(), distribution.getN());

  distribution.reset();
  EXPECT_EQ(0, distribution.getN());
  for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
    EXPECT_EQ(i, distribution.getN());
    Eigen::Matrix<float,2,1> data;
    data[0] = static_cast<float>(test_distribution_500.data[i][0]);
    data[1] = static_cast<float>(test_distribution_500.data[i][1]);
    distribution.add(data);
  }
  EXPECT_EQ(test_distribution_500.data.size(), distribution.getN());

  distribution.reset();
  EXPECT_EQ(0, distribution.getN());
  for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
    EXPECT_EQ(i, distribution.getN());
    Eigen::Matrix<float,2,1> data;
    data[0] = static_cast<float>(test_distribution_5000.data[i][0]);
    data[1] = static_cast<float>(test_distribution_5000.data[i][1]);
    distribution.add(data);
  }
  EXPECT_EQ(test_distribution_5000.data.size(), distribution.getN());

}

TEST(Test_cslibs_math, testDistributionMean)
{
  const float tolerance = 1e-3;

  cs::StableDistribution<float,2> distribution;
  Eigen::Vector2f mean;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution.add(data);
  }

  mean = distribution.getMean();

  EXPECT_NEAR(test_distribution_200.mean(0), mean(0), tolerance);
  EXPECT_NEAR(test_distribution_200.mean(1), mean(1), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_500.data[i][0]);
      data[1] = static_cast<float>(test_distribution_500.data[i][1]);
      distribution.add(data);
  }

  mean = distribution.getMean();
  EXPECT_NEAR(test_distribution_500.mean(0), mean(0), tolerance);
  EXPECT_NEAR(test_distribution_500.mean(1), mean(1), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_5000.data[i][0]);
      data[1] = static_cast<float>(test_distribution_5000.data[i][1]);
      distribution.add(data);
  }

  mean = distribution.getMean();
  EXPECT_NEAR(test_distribution_5000.mean(0), mean(0), tolerance);
  EXPECT_NEAR(test_distribution_5000.mean(1), mean(1), tolerance);
}

TEST(Test_cslibs_math, testDistributionCovariance)
{
  const float tolerance = 1e-3;

  cs::StableDistribution<float,2> distribution;
  Eigen::Matrix2f cov;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution.add(data);
  }

  cov = distribution.getCovariance();

  EXPECT_NEAR(test_distribution_200.covariance(0,0), cov(0,0), tolerance);
  EXPECT_NEAR(test_distribution_200.covariance(0,1), cov(0,1), tolerance);
  EXPECT_NEAR(test_distribution_200.covariance(1,0), cov(1,0), tolerance);
  EXPECT_NEAR(test_distribution_200.covariance(1,1), cov(1,1), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_500.data[i][0]);
      data[1] = static_cast<float>(test_distribution_500.data[i][1]);
      distribution.add(data);
  }

  cov = distribution.getCovariance();
  EXPECT_NEAR(test_distribution_500.covariance(0,0), cov(0,0), tolerance);
  EXPECT_NEAR(test_distribution_500.covariance(0,1), cov(0,1), tolerance);
  EXPECT_NEAR(test_distribution_500.covariance(1,0), cov(1,0), tolerance);
  EXPECT_NEAR(test_distribution_500.covariance(1,1), cov(1,1), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_5000.data[i][0]);
      data[1] = static_cast<float>(test_distribution_5000.data[i][1]);
      distribution.add(data);
  }

  cov = distribution.getCovariance();
  EXPECT_NEAR(test_distribution_5000.covariance(0,0), cov(0,0), tolerance);
  EXPECT_NEAR(test_distribution_5000.covariance(0,1), cov(0,1), tolerance);
  EXPECT_NEAR(test_distribution_5000.covariance(1,0), cov(1,0), tolerance);
  EXPECT_NEAR(test_distribution_5000.covariance(1,1), cov(1,1), tolerance);
}

TEST(Test_cslibs_math, testDistributionEigenValues)
{
  const float tolerance = 1e-3;

  cs::StableDistribution<float,2> distribution;
  Eigen::Vector2f eigen_values;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution.add(data);
  }

  eigen_values = distribution.getEigenValues();

  EXPECT_NEAR(test_distribution_200.eigen_values(0), eigen_values(0), tolerance);
  EXPECT_NEAR(test_distribution_200.eigen_values(1), eigen_values(1), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_500.data[i][0]);
      data[1] = static_cast<float>(test_distribution_500.data[i][1]);
      distribution.add(data);
  }

  eigen_values = distribution.getEigenValues();
  EXPECT_NEAR(test_distribution_500.eigen_values(0), eigen_values(1), tolerance);
  EXPECT_NEAR(test_distribution_500.eigen_values(1), eigen_values(0), tolerance);

  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_5000.data[i][0]);
      data[1] = static_cast<float>(test_distribution_5000.data[i][1]);
      distribution.add(data);
  }

  eigen_values = distribution.getEigenValues();
  EXPECT_NEAR(test_distribution_5000.eigen_values(0), eigen_values(0), tolerance);
  EXPECT_NEAR(test_distribution_5000.eigen_values(1), eigen_values(1), tolerance);
}

TEST(Test_cslibs_math, testDistributionEigenVectors)
{
  auto equals = [] (const Eigen::Vector2f &a,
                    const Eigen::Vector2f &b,
                    const float eps) {

    Eigen::Matrix2f invert_direction = Eigen::Matrix2f::Identity() * -1;
    Eigen::Vector2f diff_a = a - b;
    Eigen::Vector2f diff_b = a - invert_direction * b;

    return (fabs(diff_a(0)) <= eps && fabs(diff_a(1)) <= eps) ||
        (fabs(diff_b(0)) <= eps && fabs(diff_b(1)) <= eps);
  };

  const float tolerance = 1e-3;

  cs::StableDistribution<float,2> distribution;
  Eigen::Matrix2f eigen_vectors;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution.add(data);
  }

  eigen_vectors = distribution.getEigenVectors();

  Eigen::Vector2f exp_a;
  exp_a[0] = static_cast<float>(test_distribution_200.eigen_vectors.col(0)[0]);
  exp_a[1] = static_cast<float>(test_distribution_200.eigen_vectors.col(0)[1]);
  Eigen::Vector2f exp_b;
  exp_b[0] = static_cast<float>(test_distribution_200.eigen_vectors.col(1)[0]);
  exp_b[1] = static_cast<float>(test_distribution_200.eigen_vectors.col(1)[1]);

  Eigen::Vector2f rec_a = eigen_vectors.col(0);
  Eigen::Vector2f rec_b = eigen_vectors.col(1);
  /// direction and storage order of vectors has not to be equal
  bool condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
      (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

  EXPECT_TRUE(condition);

  /// 500
  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_500.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_500.data[i][0]);
      data[1] = static_cast<float>(test_distribution_500.data[i][1]);
      distribution.add(data);
  }

  eigen_vectors = distribution.getEigenVectors();

  exp_a[0] = static_cast<float>(test_distribution_500.eigen_vectors.col(0)[0]);
  exp_a[1] = static_cast<float>(test_distribution_500.eigen_vectors.col(0)[1]);
  exp_b[0] = static_cast<float>(test_distribution_500.eigen_vectors.col(1)[0]);
  exp_b[1] = static_cast<float>(test_distribution_500.eigen_vectors.col(1)[1]);

  rec_a = eigen_vectors.col(0);
  rec_b = eigen_vectors.col(1);
  /// direction and storage order of vectors has not to euqal
  condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
      (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

  EXPECT_TRUE(condition);



  /// 5000
  distribution.reset();
  for(std::size_t i = 0 ; i < test_distribution_5000.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_5000.data[i][0]);
      data[1] = static_cast<float>(test_distribution_5000.data[i][1]);
      distribution.add(data);
  }

  eigen_vectors = distribution.getEigenVectors();

  exp_a[0] = static_cast<float>(test_distribution_5000.eigen_vectors.col(0)[0]);
  exp_a[1] = static_cast<float>(test_distribution_5000.eigen_vectors.col(0)[1]);
  exp_b[0] = static_cast<float>(test_distribution_5000.eigen_vectors.col(1)[0]);
  exp_b[1] = static_cast<float>(test_distribution_5000.eigen_vectors.col(1)[1]);

  rec_a = eigen_vectors.col(0);
  rec_b = eigen_vectors.col(1);
  /// direction and storage order of vectors has not to euqal
  condition = (equals(exp_a, rec_a, tolerance) && equals(exp_b, rec_b, tolerance)) ||
      (equals(exp_a, rec_b, tolerance) && equals(exp_b, rec_a, tolerance));

  EXPECT_TRUE(condition);

}

TEST(Test_cslibs_math, testDistributionCopy)
{
  cs::StableDistribution<float,2> distribution_a;
  cs::StableDistribution<float,2> distribution_b;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution_a.add(data);
  }

  distribution_b = distribution_a;

  EXPECT_TRUE(distribution_a.getMean() == distribution_b.getMean());
  EXPECT_TRUE(distribution_a.getN()    == distribution_b.getN());

  EXPECT_TRUE(distribution_a.getCovariance() == distribution_b.getCovariance());
  EXPECT_TRUE(distribution_a.getInformationMatrix()    == distribution_b.getInformationMatrix());

  EXPECT_TRUE(distribution_a.getEigenValues() == distribution_b.getEigenValues());
  EXPECT_TRUE(distribution_a.getEigenVectors()   == distribution_b.getEigenVectors());
}


TEST(Test_cslibs_math, testDistributionAddition)
{
  const float tolerance = 1e-3;

  cs::StableDistribution<float,2> distribution_a;
  cs::StableDistribution<float,2> distribution_b;
  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution_a.add(data);
  }

  distribution_b  = distribution_a;
  distribution_b += distribution_a;

  for(std::size_t i = 0 ; i < test_distribution_200.data.size() ; ++i) {
      Eigen::Matrix<float,2,1> data;
      data[0] = static_cast<float>(test_distribution_200.data[i][0]);
      data[1] = static_cast<float>(test_distribution_200.data[i][1]);
      distribution_a.add(data);
  }

  EXPECT_NEAR((distribution_a.getMean() - distribution_b.getMean()).norm(), 0.0, tolerance);
  EXPECT_TRUE(distribution_a.getN() == distribution_b.getN());

  EXPECT_NEAR((distribution_a.getCovariance() - distribution_b.getCovariance()).norm(), 0.0, tolerance);
  EXPECT_NEAR((distribution_a.getInformationMatrix() - distribution_b.getInformationMatrix()).norm(), 0.0, tolerance);

  EXPECT_NEAR((distribution_a.getEigenValues() - distribution_b.getEigenValues()).norm(), 0.0, tolerance);
  EXPECT_NEAR((distribution_a.getEigenVectors() - distribution_b.getEigenVectors()).norm(), 0.0, tolerance);

  auto test = [tolerance](const cslibs_math::TestDistribution<2> &t) {
    cs::StableDistribution<float,2> wa;
    cs::StableDistribution<float,2> wb;

    const std::size_t middle = t.data.size() / 2 + 1;
    for(std::size_t i = 0 ; i < middle ; ++i) {
        Eigen::Matrix<float,2,1> data;
        data[0] = static_cast<float>(t.data[i][0]);
        data[1] = static_cast<float>(t.data[i][1]);
        wa.add(data);
    }
    for(std::size_t i = middle ; i < t.data.size() ; ++i) {
        Eigen::Matrix<float,2,1> data;
        data[0] = static_cast<float>(t.data[i][0]);
        data[1] = static_cast<float>(t.data[i][1]);
        wb.add(data);
    }

    wa += wb;

    auto mean = wa.getMean();
    auto cov  = wa.getCovariance();

    EXPECT_NEAR(t.mean(0), mean(0), tolerance);
    EXPECT_NEAR(t.mean(1), mean(1), tolerance);
    EXPECT_NEAR(t.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(t.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(t.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(t.covariance(1,1), cov(1,1), tolerance);

  };

  test(test_distribution_200);
  test(test_distribution_500);
  test(test_distribution_5000);
}

TEST(Test_cslibs_math, testWeightedDistribution)
{
  cs::WeightedDistribution<float,2> wd;
  cs::StableWeightedDistribution<float, 2> swd;
  for(std::size_t i = 0 ; i < 10 ; ++i) {
    wd.add(Eigen::Vector2f(i,i), 0.5);
    swd.add(Eigen::Vector2f(i,i), 0.5);
  }

  auto mean = wd.getMean();
  auto stable_mean = swd.getMean();
  EXPECT_NEAR(4.5, mean(0), 1e-6);
  EXPECT_NEAR(4.5, mean(1), 1e-6);
  EXPECT_NEAR(4.5, stable_mean(0), 1e-6);
  EXPECT_NEAR(4.5, stable_mean(1), 1e-6);

  for(std::size_t i = 0 ; i < 10 ; ++i) {
    wd.add(Eigen::Vector2f(i,i), 1.0);
    swd.add(Eigen::Vector2f(i,i), 1.0);
  }

  mean = wd.getMean();
  stable_mean = swd.getMean();

  EXPECT_NEAR(4.5, mean(0), 1e-6);
  EXPECT_NEAR(4.5, mean(1), 1e-6);
  EXPECT_NEAR(4.5, stable_mean(0), 1e-6);
  EXPECT_NEAR(4.5, stable_mean(1), 1e-6);

  for(std::size_t i = 0 ; i < 10 ; ++i) {
    wd.add(Eigen::Vector2f(2,2), 1.0);
    swd.add(Eigen::Vector2f(2,2), 1.0);
  }

  mean = wd.getMean();
  stable_mean = swd.getMean();
  EXPECT_NEAR(3.5, mean(0), 1e-6);
  EXPECT_NEAR(3.5, mean(1), 1e-6);
  EXPECT_NEAR(3.5, stable_mean(0), 1e-6);
  EXPECT_NEAR(3.5, stable_mean(1), 1e-6);


  cs::WeightedDistribution<float,2> wdc = wd;
  cs::StableWeightedDistribution<float, 2> swdc = swd;
  wdc += wd;
  swdc += swd;
  mean = wdc.getMean();
  stable_mean = swdc.getMean();
  EXPECT_NEAR(3.5, mean(0), 1e-6);
  EXPECT_NEAR(3.5, mean(1), 1e-6);
  EXPECT_NEAR(3.5, stable_mean(0), 1e-6);
  EXPECT_NEAR(3.5, stable_mean(1), 1e-6);

  wdc = wd;
  swdc = swd;
  for(std::size_t i = 0 ; i < 10 ; ++i) {
    wdc.add(Eigen::Vector2f(8,8), 0.75);
    swdc.add(Eigen::Vector2f(8,8), 0.75);
  }
  wdc += wd;
  swdc += swd;
  mean = wdc.getMean();
  stable_mean = swdc.getMean();
  EXPECT_NEAR(4.086956521739131, mean(0), 1e-6);
  EXPECT_NEAR(4.086956521739131, mean(1), 1e-6);
  EXPECT_NEAR(4.086956521739131, stable_mean(0), 1e-6);
  EXPECT_NEAR(4.086956521739131, stable_mean(1), 1e-6);
}

template<class Distribution>
void testWeightedDistributionAddition(const cslibs_math::TestDistribution<2> &t,
                                      const double tolerance = 1e-3)
{
    Distribution wa;
    Distribution wb;
    Distribution wc;
    Distribution wd;

    auto cast = [](const Eigen::Vector2d &v)
    {
        return Eigen::Vector2f(v(0), v(1));
    };


    const std::size_t middle = t.data.size() / 2 + 1;
    for(std::size_t i = 0 ; i < middle ; ++i) {
      wa.add(cast(t.data[i]), 1.0f);
      wc.add(cast(t.data[i]), 1.0f);
    }
    for(std::size_t i = middle ; i < t.data.size() ; ++i) {
      wb.add(cast(t.data[i]), 1.0f);
      wc.add(cast(t.data[i]), 1.0f);
    }

    for(std::size_t i = t.data.size() ; i > 0 ; --i) {
      wd.add(cast(t.data[i]), 1.0f);
    }

    wa += wb;

    auto mean = wc.getMean();
    auto cov  = wc.getCovariance();

    auto mean1 = wd.getMean();
    auto cov1  = wd.getCovariance();

    EXPECT_NEAR(t.mean(0), mean(0), tolerance);
    EXPECT_NEAR(t.mean(1), mean(1), tolerance);
    EXPECT_NEAR(t.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(t.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(t.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(t.covariance(1,1), cov(1,1), tolerance);

    EXPECT_NEAR(mean1(0), mean(0), tolerance);
    EXPECT_NEAR(mean1(1), mean(1), tolerance);
    EXPECT_NEAR(cov1(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(cov1(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(cov1(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(cov1(1,1), cov(1,1), tolerance);
}

template<class Distribution>
void testWeightedDistributionScale(const cslibs_math::TestDistribution<2> &t,
                                   const double tolerance = 1e-3)
{

    auto cast = [](const Eigen::Vector2d &v)
    {
        return Eigen::Vector2f(v(0), v(1));
    };

    Distribution w;

    const float weight = 1.0f / (t.data.size() + 1.0f);
    for(std::size_t i = 0 ; i < t.data.size() ; ++i) {
      w.add(cast(t.data[i]), weight);
    }

    auto mean = w.getMean();
    auto cov  = w.getCovariance();

    EXPECT_NEAR(t.mean(0), mean(0), tolerance);
    EXPECT_NEAR(t.mean(1), mean(1), tolerance);
    EXPECT_NEAR(t.covariance(0,0), cov(0,0), tolerance);
    EXPECT_NEAR(t.covariance(0,1), cov(0,1), tolerance);
    EXPECT_NEAR(t.covariance(1,0), cov(1,0), tolerance);
    EXPECT_NEAR(t.covariance(1,1), cov(1,1), tolerance);
}


TEST(Test_cslibs_math, testWeightedDistributionAddition)
{
  testWeightedDistributionAddition<cs::WeightedDistribution<float,2>>(test_distribution_200);
  testWeightedDistributionAddition<cs::WeightedDistribution<float,2>>(test_distribution_500);
  testWeightedDistributionAddition<cs::WeightedDistribution<float,2>>(test_distribution_5000);
  testWeightedDistributionAddition<cs::StableWeightedDistribution<float,2>>(test_distribution_200);
  testWeightedDistributionAddition<cs::StableWeightedDistribution<float,2>>(test_distribution_500);
  testWeightedDistributionAddition<cs::StableWeightedDistribution<float,2>>(test_distribution_5000);


  testWeightedDistributionScale<cs::WeightedDistribution<float,2>>(test_distribution_200);
  testWeightedDistributionScale<cs::WeightedDistribution<float,2>>(test_distribution_500);
  testWeightedDistributionScale<cs::WeightedDistribution<float,2>>(test_distribution_5000);
  testWeightedDistributionScale<cs::StableWeightedDistribution<float,2>>(test_distribution_200);
  testWeightedDistributionScale<cs::StableWeightedDistribution<float,2>>(test_distribution_500);
  testWeightedDistributionScale<cs::StableWeightedDistribution<float,2>>(test_distribution_5000);
}





int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cslibs_math_test_node_distribution");
  ros::NodeHandle nh_private("~");

  std::string test_distribution_200_path;
  std::string test_distribution_500_path;
  std::string test_distribution_5000_path;
  if(!nh_private.getParam("test_distribution_200", test_distribution_200_path)) {
    std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_200!" << std::endl;
    std::cout << test_distribution_200_path << std::endl;
    return -1;
  }
  if(!nh_private.getParam("test_distribution_500", test_distribution_500_path)) {
    std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_500!" << std::endl;
    return -1;
  }
  if(!nh_private.getParam("test_distribution_5000", test_distribution_5000_path)) {
    std::cerr << "[TestNodeDistribution]: Cannot load test_distribution_5000!" << std::endl;
    return -1;
  }

  test_distribution_200.read(test_distribution_200_path);
  test_distribution_500.read(test_distribution_500_path);
  test_distribution_5000.read(test_distribution_5000_path);


  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


