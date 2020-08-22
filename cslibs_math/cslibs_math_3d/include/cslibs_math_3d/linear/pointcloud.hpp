#ifndef CSLIBS_MATH_3D_POINTCLOUD_3D_HPP
#define CSLIBS_MATH_3D_POINTCLOUD_3D_HPP

#include <cslibs_math/linear/pointcloud.hpp>
#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
template <typename T>
using Pointcloud3 = typename cslibs_math::linear::Pointcloud<Point3<T>>;
using Pointcloud3d = Pointcloud3<double>;
using Pointcloud3f = Pointcloud3<float>;

template <typename T>
using PointcloudRGB3 = typename cslibs_math::linear::Pointcloud<PointRGB3<T>>;
using PointcloudRGB3d = PointcloudRGB3<double>;
using PointcloudRGB3f = PointcloudRGB3<float>;
}

#endif // CSLIBS_MATH_3D_POINTCLOUD_3D_HPP
