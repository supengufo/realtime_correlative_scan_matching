#ifndef CSLIBS_MATH_3D_LINE_3D_HPP
#define CSLIBS_MATH_3D_LINE_3D_HPP

#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
template <typename T>
using Line3 = std::array<Point3<T>, 2>;
using Line3d = Line3<double>;
using Line3f = Line3<float>;
}

#endif // CSLIBS_MATH_3D_LINE_3D_HPP
