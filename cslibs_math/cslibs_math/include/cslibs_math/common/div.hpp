#ifndef CSLIBS_MATH_DIV_HPP
#define CSLIBS_MATH_DIV_HPP

#include <type_traits>

namespace cslibs_math {
namespace common {
template<typename T>
T div(const T a, const T b)
{
    static_assert(std::is_integral<T>::value, "Integral required.");

    assert(b > T());
    const T d = a / b;
    return a < 1 ? (d*b != a ? d - 1 : d) : d;
}
}
}

#endif // CSLIBS_MATH_DIV_HPP
