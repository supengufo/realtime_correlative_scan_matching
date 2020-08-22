#ifndef CSLIBS_MATH_2D_EFLA_ITERATOR_HPP
#define CSLIBS_MATH_2D_EFLA_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
template <typename Tp = float>
class EIGEN_ALIGN16 EFLAIterator
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<EFLAIterator<Tp>>;

    using index_t       = std::array<int, 2>;

    template <typename T>
    inline explicit EFLAIterator(const Point2<T> &p0,
                                 const Point2<T> &p1,
                                 const T          &resolution) :
        EFLAIterator({{static_cast<int>(std::floor(p0(0) / resolution)),
                       static_cast<int>(std::floor(p0(1) / resolution))}},
                     {{static_cast<int>(std::floor(p1(0) / resolution)),
                       static_cast<int>(std::floor(p1(1) / resolution))}})
    {
    }

    inline explicit EFLAIterator(const index_t &start,
                                 const index_t &end) :
        start_(start),
        end_(end),
        steep_(std::abs(end[1] - start[1]) > std::abs(end[0] - start[0]))
    {
        if (steep_) {
            std::swap(start_[0], start_[1]);
            std::swap(end_[0], end_[1]);
        }
        index_ = start_;
        x_     = 0;
        y_     = start_[1];

        delta_[0] = std::abs(end_[0] - start_[0]);
        delta_[1] = std::abs(end_[1] - start_[1]);

        step_ = start_[0] < end_[0] ? 1 : -1;   // step always in x, inc in y
        inc_  = delta_[0] == 0 ? delta_[1] : (static_cast<Tp>(delta_[1])/static_cast<Tp>(delta_[0]));
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

    inline EFLAIterator& operator++()
    {
        return done() ? *this : iterate();
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) +
               sq(index_[1] - end_[1]);
    }

    inline bool done() const
    {
        return index_[0] == end_[0] &&
               index_[1] == end_[1];
    }

private:
    index_t     start_;
    index_t     end_;
    index_t     index_;
    index_t     delta_;

    bool        steep_;
    int         step_;
    Tp          inc_;
    int         x_;
    Tp          y_;

    inline EFLAIterator &iterate()
    {
        x_ += step_;
        y_ += inc_;

        index_[0] = start_[0] + x_;
        index_[1] = start_[1] + static_cast<int>(std::round(y_));
        return *this;
    }
};
}
}

#endif // CSLIBS_MATH_2D_EFLA_ITERATOR_HPP
