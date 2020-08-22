#ifndef CSLIBS_MATH_2D_LINE_2D_HPP
#define CSLIBS_MATH_2D_LINE_2D_HPP

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
template <typename T>
using Line2 = std::array<Point2<T>, 2>;
using Line2d = Line2<double>;
using Line2f = Line2<float>;
}

#endif // CSLIBS_MATH_2D_LINE_2D_HPP
