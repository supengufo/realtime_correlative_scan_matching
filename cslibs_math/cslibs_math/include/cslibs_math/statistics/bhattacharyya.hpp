#ifndef CSLIBS_MATH_BHATTACHARYYA_HPP
#define CSLIBS_MATH_BHATTACHARYYA_HPP

#include <cslibs_math/statistics/distribution.hpp>
#include <cslibs_math/statistics/weighted_distribution.hpp>

namespace cslibs_math {
namespace statistics {
template<typename T, std::size_t Dim>
inline T bhattacharyya(const Eigen::Matrix<T, Dim, Dim>   &s_a,
                       const Eigen::Matrix<T, Dim, 1>     &m_a,
                       const Eigen::Matrix<T, Dim, Dim>   &s_b,
                       const Eigen::Matrix<T, Dim, 1>     &m_b)
{
    const Eigen::Matrix<T, Dim, 1>  dm = (m_a - m_b);
    const Eigen::Matrix<T, Dim, Dim> s = (s_a + s_b) * 0.5;
    const T det_sa = s_a.determinant();
    const T det_sb = s_b.determinant();
    const T det_s  = s.determinant();

    return 0.125 * static_cast<T>(Eigen::Matrix<T, 1, Dim>(dm.transpose()) * s.inverse() * dm) +
           0.5 * std::log(det_s / std::sqrt(det_sa * det_sb));
}

template<typename T, std::size_t Dim, std::size_t lamda_ratio_exponent = 0>
inline T bhattacharyya(const Distribution<T, Dim, lamda_ratio_exponent> &a,
                       const Distribution<T, Dim, lamda_ratio_exponent> &b)
{
    return bhattacharyya<T, Dim,>(a.getCovariance(), a.getMean(),
                                  b.getCovariance(), b.getMean());
}

template<typename T, std::size_t Dim, std::size_t lamda_ratio_exponent = 0>
inline T bhattacharyya(const WeightedDistribution<T, Dim, lamda_ratio_exponent> &a,
                       const WeightedDistribution<T, Dim, lamda_ratio_exponent> &b)
{
    return bhattacharyya<T, Dim>(a.getCovariance(), a.getMean(),
                                 b.getCovariance(), b.getMean());
}
}
}

#endif // CSLIBS_MATH_BHATTACHARYYA_HPP
