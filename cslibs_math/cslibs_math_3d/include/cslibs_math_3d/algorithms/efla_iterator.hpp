#ifndef CSLIBS_MATH_3D_EFLA_ITERATOR_HPP
#define CSLIBS_MATH_3D_EFLA_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
template <typename Tp = double>
class EIGEN_ALIGN16 EFLAIterator
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<EFLAIterator<Tp>>;

    using index_t       = std::array<int, 3>;

    template <typename T>
    inline explicit EFLAIterator(const Point3<T> &p0,
                                 const Point3<T> &p1,
                                 const T         &resolution) :
        EFLAIterator({{static_cast<int>(std::floor(p0(0) / resolution)),
                       static_cast<int>(std::floor(p0(1) / resolution)),
                       static_cast<int>(std::floor(p0(2) / resolution))}},
                     {{static_cast<int>(std::floor(p1(0) / resolution)),
                       static_cast<int>(std::floor(p1(1) / resolution)),
                       static_cast<int>(std::floor(p1(2) / resolution))}})
    {
    }

    inline explicit EFLAIterator(const index_t &start,
                                 const index_t &end) :
        start_(start),
        end_(end),
        index_(start_),
        j0_(0.0),
        j1_(0.0)
    {
        int short_len  = end_[2] - start_[2];
        int middle_len = end_[1] - start_[1];
        int long_len   = end_[0] - start_[0];

        bool y_longer = false;
        bool z_longer = false;

        if (std::fabs(middle_len) > std::fabs(long_len)) {
            std::swap(middle_len, long_len);
            y_longer = true;
        }

        if (std::fabs(short_len) > std::fabs(long_len)) {
            std::swap(short_len, long_len);
            z_longer = true;
        }

        increment_val_ = std::copysign(1.0, long_len);
        dec_inc0_ = (long_len == 0) ?
                    static_cast<Tp>(middle_len) :
                    (static_cast<Tp>(middle_len) / static_cast<Tp>(std::fabs(long_len)));
        dec_inc1_ = (long_len == 0) ?
                    static_cast<Tp>(short_len) :
                    (static_cast<Tp>(short_len) / static_cast<Tp>(std::fabs(long_len)));

        if (z_longer && !y_longer)
            std::swap(dec_inc0_, dec_inc1_);
        iterate_ = z_longer ? &EFLAIterator::iterateZ : (y_longer ? &EFLAIterator::iterateY : &EFLAIterator::iterateX);
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

    inline EFLAIterator& operator++()
    {
        return done() ? *this : (this->*iterate_)();
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) +
               sq(index_[1] - end_[1]) +
               sq(index_[2] - end_[2]);
    }

    inline bool done() const
    {
        return index_[0] == end_[0] &&
               index_[1] == end_[1] &&
               index_[2] == end_[2];
    }

private:
    index_t    start_;
    index_t    end_;
    index_t    index_;
    int        increment_val_;
    Tp         j0_;
    Tp         j1_;
    Tp         dec_inc0_;
    Tp         dec_inc1_;

    EFLAIterator&   (EFLAIterator::*iterate_)();

    inline EFLAIterator &iterateX()
    {
        j0_ += dec_inc0_;
        j1_ += dec_inc1_;

        index_[0] += increment_val_;
        index_[1]  = start_[1] + static_cast<int>(std::round(j0_));
        index_[2]  = start_[2] + static_cast<int>(std::round(j1_));

        return *this;
    }

    inline EFLAIterator &iterateY()
    {
        j0_ += dec_inc0_;
        j1_ += dec_inc1_;

        index_[0]  = start_[0] + static_cast<int>(std::round(j0_));
        index_[1] += increment_val_;
        index_[2]  = start_[2] + static_cast<int>(std::round(j1_));

        return *this;
    }

    inline EFLAIterator &iterateZ()
    {
        j0_ += dec_inc0_;
        j1_ += dec_inc1_;

        index_[0]  = start_[0] + static_cast<int>(std::round(j0_));
        index_[1]  = start_[1] + static_cast<int>(std::round(j1_));
        index_[2] += increment_val_;

        return *this;
    }
};
}
}

#endif // CSLIBS_MATH_3D_EFLA_ITERATOR_HPP
