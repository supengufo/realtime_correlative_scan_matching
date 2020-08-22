#ifndef CSLIBS_MATH_FLOOR_HPP
#define CSLIBS_MATH_FLOOR_HPP

#include <type_traits>

namespace cslibs_math {
namespace common {
template<typename T>
int floor(const T x)
{
    return x > 0 ? static_cast<int>(x) : static_cast<int>(x) - 1;
}
}
}

#endif // CSLIBS_MATH_FLOOR_HPP
