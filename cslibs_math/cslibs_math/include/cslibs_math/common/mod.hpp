#ifndef CSLIBS_MATH_MOD_HPP
#define CSLIBS_MATH_MOD_HPP

#include <type_traits>
#include <assert.h>

namespace cslibs_math {
namespace common {
template<typename T>
T mod(const T a, const T b)
{
    static_assert(std::is_integral<T>::value, "Integral required.");

    assert (b > 0);
    auto r = [b](const T x) { return x < T() ? (x + b) : x; };
    return r(a % b);
}
}
}

#endif // CSLIBS_MATH_MOD_HPP
