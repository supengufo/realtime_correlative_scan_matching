#ifndef CSLIBS_MATH_2D_VECTOR_2D_HPP
#define CSLIBS_MATH_2D_VECTOR_2D_HPP

#include <cslibs_math/linear/vector.hpp>
#include <ostream>

namespace cslibs_math_2d {
template <typename T>
using Vector2 = cslibs_math::linear::Vector<T,2>;
using Vector2d = Vector2<double>;
using Vector2f = Vector2<float>;

template <typename T>
inline T angle(const Vector2<T> &v)
{
    return static_cast<T>(std::atan2(v(1), v(0)));
}
}

namespace std {
template <typename T>
inline bool isnormal(const cslibs_math_2d::Vector2<T> &v)
{
    return std::isnormal(v(0)) &&
           std::isnormal(v(1));
}
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Vector2<T> &v)
{
    out << "[" << v(0) << "," << v(1) << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_VECTOR_2D_HPP
