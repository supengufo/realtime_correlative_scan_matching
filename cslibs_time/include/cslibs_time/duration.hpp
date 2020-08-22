#ifndef CSLIBS_TIME_DURATION_HPP
#define CSLIBS_TIME_DURATION_HPP

#include <chrono>
#include <thread>
#include <cmath>
#include <ostream>

namespace cslibs_time {
class Duration {
public:
    using clock_t    = std::chrono::high_resolution_clock;
    using time_t     = clock_t::time_point;
    using duration_t = clock_t::duration;

    inline Duration() :
        duration_(0l)
    {
    }

    explicit inline Duration(const double seconds) :
        duration_(static_cast<int64_t>(std::floor(seconds * 1e9)))
    {
    }

    explicit inline Duration(const int64_t nanoseconds) :
        duration_(nanoseconds)
    {
    }

    inline Duration(const duration_t duration) :
        duration_(duration)
    {
    }

    inline double seconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count() * 1e-9;
    }

    inline double milliseconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count() * 1e-6;
    }

    inline int64_t nanoseconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count();
    }

    inline duration_t duration() const
    {
        return duration_;
    }

    inline Duration& operator += (const Duration &other)
    {
        duration_ += other.duration_;
        return *this;
    }

    inline Duration& operator -= (const Duration &other)
    {
        duration_ -= other.duration_;
        return *this;
    }

    inline bool sleep() const
    {
        if(duration_ <= duration_t(0))
            return false;
        std::this_thread::sleep_for(duration_);
        return true;
    }

    inline bool isZero() const
    {
        return duration_.count() == 0l;
    }

private:
    duration_t duration_;
};

inline cslibs_time::Duration operator + (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return cslibs_time::Duration(a.duration() + b.duration());
}

inline cslibs_time::Duration operator - (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return cslibs_time::Duration(a.duration() - b.duration());
}

inline cslibs_time::Duration operator * (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return cslibs_time::Duration(a.nanoseconds() * b.nanoseconds());
}

inline cslibs_time::Duration operator * (const cslibs_time::Duration &a, const double s)
{
    return cslibs_time::Duration(static_cast<int64_t>(std::floor(a.nanoseconds() * s)));
}

inline cslibs_time::Duration operator * (const double s, const cslibs_time::Duration &a)
{
    return cslibs_time::Duration(static_cast<int64_t>(std::floor(a.nanoseconds() * s)));
}

inline cslibs_time::Duration operator / (const cslibs_time::Duration &a, const double s)
{
    return cslibs_time::Duration(static_cast<int64_t>(std::floor(a.nanoseconds() / s)));
}

inline bool operator == (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() == b.duration();
}

inline bool operator != (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() != b.duration();
}

inline bool operator <= (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() <= b.duration();
}

inline bool operator >= (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() >= b.duration();
}

inline bool operator > (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() > b.duration();
}

inline bool operator < (const cslibs_time::Duration &a, const cslibs_time::Duration &b)
{
    return a.duration() < b.duration();
}

inline std::ostream & operator << (std::ostream &out, const cslibs_time::Duration &duration)
{
    out << "[" << std::to_string(duration.seconds()) <<  "]";
    return out;
}
}

#endif // CSLIBS_TIME_DURATION_HPP
