#ifndef CSLIBS_MATH_2D_POINTCLOUD2D_HPP
#define CSLIBS_MATH_2D_POINTCLOUD2D_HPP

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math/linear/pointcloud.hpp>

namespace cslibs_math_2d {
template <typename T>
using Pointcloud2 = typename cslibs_math::linear::Pointcloud<Point2<T>>;
using Pointcloud2d = Pointcloud2<double>;
using Pointcloud2f = Pointcloud2<float>;
}

#endif // CSLIBS_MATH_2D_POINTCLOUD2D_HPP
