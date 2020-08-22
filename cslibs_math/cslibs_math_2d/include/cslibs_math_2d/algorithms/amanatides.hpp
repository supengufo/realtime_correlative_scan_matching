#ifndef CSLIBS_MATH_2D_AMANATIDES_HPP
#define CSLIBS_MATH_2D_AMANATIDES_HPP

#include <memory>
#include <array>
#include <cmath>
#include <cslibs_math/common/equal.hpp>

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
template <typename Tp = double>
class EIGEN_ALIGN16 Amanatides
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<Amanatides<Tp>>;

    using index_t       = std::array<int, 2>;
    using delta_t       = std::array<Tp, 2>;

    inline Amanatides() :
        index_{{0,0}},
        end_{{0,0}},
        delta_{{Tp(), Tp()}},
        max_{{Tp(), Tp()}}
    {
    }

    inline explicit Amanatides(const Point2<Tp> &start,
                               const Point2<Tp> &end,
                               const Tp resolution) :
        index_{{static_cast<int>(std::floor(start(0) / resolution)),
               static_cast<int>(std::floor(start(1) / resolution))}},
        end_{{static_cast<int>(std::floor(end(0) / resolution)),
              static_cast<int>(std::floor(end(1) / resolution))}}
    {
        Point2<Tp> d = (end - start).normalized();

        const static Tp dmax = std::numeric_limits<Tp>::max();
        step_[0]       = d(0) > 0 ? 1 : (d(0) < 0 ? -1 : 0);
        step_[1]       = d(1) > 0 ? 1 : (d(1) < 0 ? -1 : 0);
        const bool dx  = (step_[0] != 0);
        const bool dy  = (step_[1] != 0);
        delta_[0]      = dx ? resolution / std::fabs(d(0)) : dmax;
        delta_[1]      = dy ? resolution / std::fabs(d(1)) : dmax;
        max_[0]        = dx ? (std::ceil(static_cast<Tp>(index_[0]) + static_cast<Tp>(step_[0]) * 0.5) * resolution - start(0)) / d(0) : dmax;
        max_[1]        = dy ? (std::ceil(static_cast<Tp>(index_[1]) + static_cast<Tp>(step_[1]) * 0.5) * resolution - start(1)) / d(1) : dmax;
    }

    inline virtual ~Amanatides()
    {
    }

    inline int x() const
    {
        return index_[0];
    }

    inline int y() const
    {
        return index_[1];
    }

    inline index_t operator()() const
    {
        return index_;
    }

    inline Amanatides& operator++()
    {
        return done() ? *this : iterate(max_[0] < max_[1] ? 0ul : 1ul);
    }

    inline bool done() const
    {
        return (index_[0] == end_[0] && index_[1] == end_[1]);
    }

private:
    inline Amanatides &iterate(const std::size_t dim)
    {
        max_[dim]   += delta_[dim];
        index_[dim] += step_[dim];
        return *this;
    }

    index_t index_;
    index_t end_;
    index_t step_;
    delta_t delta_;
    delta_t max_;
};
}
}

#endif // CSLIBS_MATH_2D_AMANATIDES_HPP
