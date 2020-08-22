#ifndef CSLIBS_MATH_3D_AMANATIDES_HPP
#define CSLIBS_MATH_3D_AMANATIDES_HPP

#include <memory>
#include <array>
#include <cmath>
#include <cslibs_math/common/equal.hpp>

#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
template <typename Tp = double>
class EIGEN_ALIGN16 Amanatides
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<Amanatides<Tp>>;

    using index_t       = std::array<int, 3>;
    using delta_t       = std::array<Tp, 3>;

    inline Amanatides() :
        index_{{0, 0, 0}},
        end_{{0, 0, 0}},
        delta_{{Tp(), Tp(), Tp()}},
        max_{{Tp(), Tp(), Tp()}}
    {
    }

    inline explicit Amanatides(const Point3<Tp> &start,
                               const Point3<Tp> &end,
                               const Tp          resolution) :
        index_{{static_cast<int>(std::floor(start(0) / resolution)),
                static_cast<int>(std::floor(start(1) / resolution)),
                static_cast<int>(std::floor(start(2) / resolution))}},
        end_{{static_cast<int>(std::floor(end(0) / resolution)),
              static_cast<int>(std::floor(end(1) / resolution)),
              static_cast<int>(std::floor(end(2) / resolution))}}
    {
        Point3<Tp> d = (end - start).normalized();

        const static Tp dmax =  std::numeric_limits<Tp>::max();
        step_[0]       = d(0) > 0 ? 1 : (d(0) < 0 ? -1 : 0);
        step_[1]       = d(1) > 0 ? 1 : (d(1) < 0 ? -1 : 0);
        step_[2]       = d(2) > 0 ? 1 : (d(2) < 0 ? -1 : 0);
        const bool dx  = (step_[0] != 0);
        const bool dy  = (step_[1] != 0);
        const bool dz  = (step_[2] != 0);
        delta_[0]      = dx ? resolution / std::fabs(d(0)) : dmax;
        delta_[1]      = dy ? resolution / std::fabs(d(1)) : dmax;
        delta_[2]      = dz ? resolution / std::fabs(d(2)) : dmax;
        max_[0]        = dx ? (std::ceil(static_cast<Tp>(index_[0]) + static_cast<Tp>(step_[0]) * 0.5) * resolution - start(0)) / d(0) : dmax;
        max_[1]        = dy ? (std::ceil(static_cast<Tp>(index_[1]) + static_cast<Tp>(step_[1]) * 0.5) * resolution - start(1)) / d(1) : dmax;
        max_[2]        = dz ? (std::ceil(static_cast<Tp>(index_[2]) + static_cast<Tp>(step_[2]) * 0.5) * resolution - start(2)) / d(2) : dmax;
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

    inline int z() const
    {
        return index_[2];
    }

    inline index_t operator()() const
    {
        return index_;
    }

    inline Amanatides& operator++()
    {
        std::size_t dim =  max_[0] < max_[1] ?
                          (max_[0] < max_[2] ? 0ul : 2ul) :
                          (max_[1] < max_[2] ? 1ul : 2ul) ;

        return done() ? *this : iterate(dim);
    }

    inline bool done() const
    {
        return (index_[0] == end_[0] &&
                index_[1] == end_[1] &&
                index_[2] == end_[2]);
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

#endif // CSLIBS_MATH_3D_AMANATIDES_HPP
