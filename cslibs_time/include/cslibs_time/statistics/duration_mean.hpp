#ifndef CSLIBS_TIME_STASTICS_DURATION_HPP
#define CSLIBS_TIME_STASTICS_DURATION_HPP

#include <cslibs_time/duration.hpp>

namespace cslibs_time {
namespace statistics {
class DurationMean {
public:
    using duration_t = cslibs_time::Duration;

    inline DurationMean() :
        n_(1),
        n_1_(0)
    {
    }

    inline DurationMean& operator += (const duration_t &d)
    {
        const duration_t mean_1 = mean_;
        const double n = static_cast<double>(n_);
        const double n_1 = static_cast<double>(n_1_);

        mean_      = (mean_ * n_1 + d) / n;
        deviation_ = (deviation_ * n_1 + (d - mean_1) * (d - mean_)) / n;
        ++n_;
        ++n_1_;
        return *this;
    }

    inline duration_t const & mean() const
    {
        return mean_;
    }

    inline duration_t const &variance() const
    {
        return deviation_;
    }

private:
    std::size_t n_;
    std::size_t n_1_;

    duration_t mean_;
    duration_t deviation_;
};
}
}

#endif // CSLIBS_TIME_STASTICS_DURATION_HPP
