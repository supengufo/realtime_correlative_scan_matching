#ifndef CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP
#define CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
template <typename T>
class EIGEN_ALIGN16 SimpleIterator
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<SimpleIterator<T>>;

    using index_t       = std::array<int, 3>;
    using point_t       = Point3<T>;

    inline explicit SimpleIterator(const index_t &start,
                                   const index_t &end) :
        SimpleIterator(point_t(start[0], start[1], start[2]),
                       point_t(end[0], end[1], end[2]), 1.0)
    {
    }

    inline explicit SimpleIterator(const point_t &p0,
                                   const point_t &p1,
                                   const T       &resolution) :
        index_({{static_cast<int>(std::floor(p0(0) / resolution)),
                 static_cast<int>(std::floor(p0(1) / resolution)),
                 static_cast<int>(std::floor(p0(2) / resolution))}}),
        end_({{static_cast<int>(std::floor(p1(0) / resolution)),
               static_cast<int>(std::floor(p1(1) / resolution)),
               static_cast<int>(std::floor(p1(2) / resolution))}}),
        resolution_inv_(1.0/resolution),
        diff_((p1-p0)),
        point_(p0)
    {
        N_ = static_cast<int>(diff_.length() * resolution_inv_);
        diff_ /= static_cast<T>(N_);
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

    inline SimpleIterator& operator++()
    {
        if (done())
            return *this;

        index_t i;
        do {
            i = index_;
            point_ += diff_;
            index_[0] = cslibs_math::common::floor(point_(0) * resolution_inv_);
            index_[1] = cslibs_math::common::floor(point_(1) * resolution_inv_);
            index_[2] = cslibs_math::common::floor(point_(2) * resolution_inv_);
        } while (i[0] == index_[0] && i[1] == index_[1] && i[2] == index_[2]);
        --N_;

        return *this;
    }

    inline bool done() const
    {
        return (index_[0] == end_[0] &&
                index_[1] == end_[1] &&
                index_[2] == end_[2]) ||
                N_ < 0;
    }

private:
    index_t index_;
    index_t end_;
    T       resolution_inv_;
    point_t diff_;
    point_t point_;
    int     N_;
};
}
}

#endif // CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP
