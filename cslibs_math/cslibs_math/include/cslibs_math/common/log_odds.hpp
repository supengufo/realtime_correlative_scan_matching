#ifndef CSLIBS_MATH_LOG_ODDS_HPP
#define CSLIBS_MATH_LOG_ODDS_HPP

#include <cmath>

namespace cslibs_math {
namespace common {
template <typename T>
struct LogOdds {
    inline static T to(const T p)
    {
        return std::log(p / (1.0 - p));
    }

    inline static T from(const T l)
    {
        return 1.0 / (1.0 + std::exp(-l));
    }
};
}
}

#endif // LOG_ODDS_HPP
