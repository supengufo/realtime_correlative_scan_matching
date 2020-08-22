#ifndef CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP
#define CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP

#include <cslibs_math/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/polar_point.hpp>

namespace cslibs_math_2d {
template <typename T>
using PolarPointcloud2 = cslibs_math::linear::Pointcloud<PolarPoint2<T>>;
using PolarPointcloud2d = PolarPointcloud2<double>;
using PolarPointcloud2f = PolarPointcloud2<float>;
}
#endif // CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP
