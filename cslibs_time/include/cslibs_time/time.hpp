#ifndef CSLIBS_TIME_TIME_HPP
#define CSLIBS_TIME_TIME_HPP

#include <cslibs_time/duration.hpp>

#include <ostream>
#include <ctime>

namespace cslibs_time {
class Time {
public:
    using clock_t    = std::chrono::high_resolution_clock;
    using time_t     = clock_t::time_point;
    using duration_t = clock_t::duration;

    inline Time() :
        time_(duration_t(0))
    {
    }

    inline Time(const double seconds) :
        time_(std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9)))
    {
    }

    inline Time(const int64_t &nanoseconds) :
        time_(duration_t(nanoseconds))
    {
    }

    inline Time(const uint64_t &nanoseconds) :
        time_(duration_t(static_cast<int64_t>(nanoseconds)))
    {
    }

    inline Time(const time_t &time) :
        time_(time)
    {
    }

    inline time_t const & time() const
    {
        return time_;
    }

    inline double seconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time_.time_since_epoch()).count() * 1e-9;
    }

    inline int64_t nanoseconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time_.time_since_epoch()).count();
    }

    inline bool isZero() const
    {
        return time_ == time_t(duration_t(0));
    }

    inline Time static now()
    {
        return Time(clock_t::now());
    }

private:
    time_t time_;
};

inline bool operator == (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
    return a.time() == b.time();
}

inline bool operator != (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
    return a.time() != b.time();
}

inline bool operator <= (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
    return a.time() <= b.time();
}

inline bool operator >= (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
     return a.time() >= b.time();
}

inline bool operator > (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
     return a.time() > b.time();
}

inline bool operator < (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
     return a.time() < b.time();
}

inline cslibs_time::Duration operator - (const cslibs_time::Time &a, const cslibs_time::Time &b)
{
    return cslibs_time::Duration(a.time() - b.time());
}

inline cslibs_time::Time operator - (const cslibs_time::Time &t, const cslibs_time::Duration &d)
{
    return cslibs_time::Time(t.time() - d.duration());
}

inline cslibs_time::Time operator + (const cslibs_time::Time &t, const cslibs_time::Duration &d)
{
    return cslibs_time::Time(t.time() + d.duration());
}

inline std::ostream & operator << (std::ostream &out, const cslibs_time::Time &time)
{
    const int64_t ns = time.nanoseconds();
    const int64_t s = ns / static_cast<int64_t>(1e9);
    const int64_t ms = (ns % static_cast<int64_t>(1e9));
    out << "<" << std::to_string(s) + "." + std::to_string(ms) << ">";
    return out;
}
}

#endif // CSLIBS_TIME_TIME_HPP
