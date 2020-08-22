#ifndef CSLIBS_MATH_SQRT_HPP
#define CSLIBS_MATH_SQRT_HPP

#include <limits>

namespace cslibs_math {
namespace common {
template <typename T>
constexpr T sqrtNewtonRaphson(T x, T curr, T prev)
{
    return curr == prev ? curr : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}

template <typename T>
constexpr T sqrt(T x)
{
    return x >= 0 && x < std::numeric_limits<T>::infinity() ?
        sqrtNewtonRaphson<T>(x, x, 0) : std::numeric_limits<T>::quiet_NaN();
}
}
}

#endif // CSLIBS_MATH_SQRT_HPP
