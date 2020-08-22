#ifndef CSLIBS_TIME_RATE_HPP
#define CSLIBS_TIME_RATE_HPP

#include <cslibs_time/time.hpp>
#include <cslibs_time/duration.hpp>

namespace cslibs_time {
class Rate {
public:
    using clock_t    = std::chrono::high_resolution_clock;
    using time_t     = clock_t::time_point;
    using duration_t = clock_t::duration;

    inline Rate() :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(std::numeric_limits<double>::infinity())
    {
    }

    explicit inline Rate(const double rate) :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(1.0 / rate)
    {
    }

    explicit inline Rate(const Duration &d) :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(d)
    {
    }

    inline void reset()
    {
        start_ = Time::now();
    }

    inline Duration cycleTime() const
    {
        return actual_cycle_time_;
    }

    inline Duration expectedCycleTime() const
    {
        return expected_cycle_time_;
    }

    inline double frequency() const
    {
        return 1.0 / expectedCycleTime().seconds();
    }

    inline bool sleep()
    {
        Time expected_end = start_ + expected_cycle_time_;
        Time actual_end = Time::now();

        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }

        Duration sleep_duration = expected_end - actual_end;
        actual_cycle_time_ = actual_end - start_;
        start_ = expected_end;

        if(sleep_duration <= Duration(0.0))
        {
            if (actual_end > expected_end + expected_cycle_time_)
            {
                start_ = actual_end;
            }
            return false;
        }

        return sleep_duration.sleep();
    }
private:
    Time     start_;
    Duration actual_cycle_time_;
    Duration expected_cycle_time_;
};
}


#endif // CSLIBS_TIME_RATE_HPP
