#ifndef CSLIBS_TIME_STASTICS_DURATION_WINDOW_HPP
#define CSLIBS_TIME_STASTICS_DURATION_WINDOW_HPP

#include <cslibs_time/duration.hpp>

namespace cslibs_time {
namespace statistics {
class DurationWindow {
public:
    using duration_t = cslibs_time::Duration;

    inline DurationWindow() :
        n_(1),
        n_1_(0),
        size_(std::numeric_limits<std::size_t>::max())
    {
    }

    inline DurationWindow(const std::size_t size) :
        n_(1),
        n_1_(0),
        size_(size)
    {
    }

    inline DurationWindow& operator += (const duration_t &d)
    {

        const double n = static_cast<double>(n_);
        const double n_1 = static_cast<double>(n_1_);

        auto update_Window = [this, n, n_1] (const duration_t &d) ->DurationWindow&
        {
            const duration_t mean_1 = mean_ * n - oldest_;
            const duration_t deviation_1 = deviation_ * n - (oldest_ - mean_1) * (oldest_ - mean_);
            mean_      = (mean_1 + d) / n;
            deviation_ = (deviation_1 + (d - mean_1) * (d - mean_)) / n;
            return *this;
        };
        auto update = [this, n, n_1](const duration_t &d) ->DurationWindow&
        {
            const duration_t mean_1 = mean_;
            mean_      = (mean_ * n_1 + d) / n;
            deviation_ = (deviation_ * n_1 + (d - mean_1) * (d - mean_)) / n;
            ++n_;
            ++n_1_;
            return *this;
        };

        return n_ >= size_ ? update_Window(d) : update(d);
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
    std::size_t size_;

    duration_t mean_;
    duration_t deviation_;
    duration_t oldest_;
};
}
}

#endif // CSLIBS_TIME_STASTICS_DURATION_WINDOW_HPP
