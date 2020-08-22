#ifndef CSLIBS_MATH_2D_BRESENHAM_HPP
#define CSLIBS_MATH_2D_BRESENHAM_HPP

#include <memory>
#include <array>

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
class EIGEN_ALIGN16 Bresenham
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<Bresenham>;

    using index_t       = std::array<int, 2>;

    inline Bresenham() :
        index_{{0,0}},
        end_{{0,0}}
    {
    }

    template <typename T>
    inline explicit Bresenham(const Point2<T> &p0,
                              const Point2<T> &p1,
                              const T resolution):
    Bresenham({{static_cast<int>(std::floor(p0(0) / resolution)),
                static_cast<int>(std::floor(p0(1) / resolution))}},
              {{static_cast<int>(std::floor(p1(0) / resolution)),
                static_cast<int>(std::floor(p1(1) / resolution))}})
    {
    }

    inline explicit Bresenham(const index_t &start,
                              const index_t &end) :
        index_(start),
        end_(end),
        steep_(std::abs(end[1] - start[1]) > std::abs(end[0] - start[0])),
        error_(0)
    {
        if (steep_) {
            std::swap(index_[0], index_[1]);
            std::swap(end_[0], end_[1]);
        }

        delta_[0] = std::abs(end_[0] - index_[0]);
        delta_[1] = std::abs(end_[1] - index_[1]);

        step_[0]  = index_[0] < end_[0] ? 1 : -1;
        step_[1]  = index_[1] < end_[1] ? 1 : -1;
    }

    inline virtual ~Bresenham()
    {
    }

    inline int x() const
    {
        return (steep_ ? index_[1] : index_[0]);
    }

    inline int y() const
    {
        return (steep_ ? index_[0] : index_[1]);
    }

    inline index_t operator()() const
    {
        return {{x(), y()}};
    }

    inline Bresenham& operator++()
    {
        return done() ? *this : iterate();
    }

    inline bool done() const
    {
        return index_[0] == end_[0] && index_[1] == end_[1];
    }

private:
    inline Bresenham &iterate()
    {
        index_[0] += step_[0];
        error_    += delta_[1];
        if (2 * error_ >= delta_[0]) {
            index_[1] += step_[1];
            error_    -= delta_[0];
        }
        return *this;
    }

    index_t index_;
    index_t end_;
    index_t step_;
    index_t delta_;

    bool    steep_;
    int     error_;
};
}
}

#endif // CSLIBS_MATH_2D_BRESENHAM_HPP
