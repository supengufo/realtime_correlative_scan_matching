#ifndef CSLIBS_MATH_EQUAL_HPP
#define CSLIBS_MATH_EQUAL_HPP

#include <limits>
#include <cmath>

namespace cslibs_math {
namespace common {
template<typename T>
inline bool eq(const T a, const T b, const T eps = std::numeric_limits<T>::epsilon())
{
    return std::abs(a - b) < eps;
}

template<typename T>
inline bool ge(const T a, const T b, const T eps = std::numeric_limits<T>::epsilon())
{
    return eq(a,b, eps) || a > b;
}

template<typename T>
inline bool le(const T a, const T b, const T eps = std::numeric_limits<T>::epsilon())
{
    return eq(a,b, eps) || a < b;
}

template<typename T>
inline bool neq(const T a, const T b, const T eps = std::numeric_limits<T>::epsilon())
{
    return !eq(a,b, eps);
}
}
}

#endif // EQUAL_HPP
