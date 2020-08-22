#ifndef CSLIBS_TIME_STASTICS_DURATION_LOWPASS_HPP
#define CSLIBS_TIME_STASTICS_DURATION_LOWPASS_HPP

#include <cslibs_time/duration.hpp>

namespace cslibs_time {
namespace statistics {
class DurationLowpass {
public:
    using duration_t = cslibs_time::Duration;

    inline DurationLowpass(const double alpha = 0.01) :
        alpha_(alpha)
    {
    }

    inline DurationLowpass& operator += (const duration_t &d)
    {
        duration_ = d * alpha_ + duration_ * (1.0 - alpha_);
        return *this;
    }

    inline duration_t const & duration() const
    {
        return duration_;
    }

private:
    double     alpha_;
    duration_t duration_;
};
}
}

#endif // CSLIBS_TIME_STASTICS_DURATION_LOWPASS_HPP
