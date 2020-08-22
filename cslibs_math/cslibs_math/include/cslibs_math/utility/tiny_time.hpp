#ifndef CSLIBS_MATH_TINY_TIME_HPP
#define CSLIBS_MATH_TINY_TIME_HPP

#include <chrono>

namespace cslibs_math {
namespace utility {
namespace tiny_time {
using clock_t    = std::chrono::high_resolution_clock;
using time_t     = clock_t::time_point;
using duration_t = clock_t::duration;


inline time_t fromMilliseconds(const int64_t n)
{
    return time_t(std::chrono::milliseconds(n));
}

inline time_t fromMicroseconds(const int64_t n)
{
    return time_t(std::chrono::microseconds(n));
}

inline time_t fromNanoseconds(const int64_t n)
{
    return time_t(std::chrono::nanoseconds(n));
}

template <typename T = double>
inline T seconds(const time_t &time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count() * 1e-9;
}

template <typename T = double>
inline T milliseconds(const time_t &time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count() * 1e-6;
}

template <typename T = double>
inline T microseconds(const time_t &time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count() * 1e-3;
}

template <typename T = double>
inline T nanoseconds(const time_t &time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

template <typename T = double>
inline T seconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-9;
}

template <typename T = double>
inline T milliseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-6;
}

template <typename T = double>
inline T microseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-3;
}

inline int64_t nanoseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}
}
}
}

#endif // CSLIBS_MATH_TINY_TIME_HPP
