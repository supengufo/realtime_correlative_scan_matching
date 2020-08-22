#ifndef CSLIBS_MATH_STAMPED_HPP
#define CSLIBS_MATH_STAMPED_HPP

#include <memory>

#include <eigen3/Eigen/Core>

#include "tiny_time.hpp"

namespace cslibs_math {
namespace utility {
template<typename T>
class EIGEN_ALIGN16 Stamped
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t   = Eigen::aligned_allocator<Stamped<T>>;

    using Ptr           = std::shared_ptr<Stamped<T>>;
    using ConstPtr      = std::shared_ptr<const Stamped<T>>;
    using time_t        = tiny_time::time_t;

    explicit inline Stamped(const time_t &time) :
        time_(time)
    {
    }

    inline explicit Stamped(const T &data,
                            const time_t &time) :
        data_(data),
        time_(time)
    {
    }

    inline Stamped(const Stamped<T> &other) :
        data_(other.data_),
        time_(other.time_)
    {
    }

    inline Stamped(Stamped<T> &&other) :
        data_(std::move(other.data_)),
        time_(other.time_)
    {

    }

    inline Stamped<T>& operator = (const Stamped<T> &other)
    {
        time_ = other.time_;
        data_ = other.data_;
        return *this;
    }

    inline Stamped<T>& operator = (Stamped<T> &&other)
    {
        time_ = other.time_;
        data_ = std::move(other.data_);
        return *this;
    }

    inline time_t & stamp()
    {
        return time_;
    }

    inline time_t const & stamp() const
    {
        return time_;
    }

    inline T & data()
    {
        return data_;
    }

    inline T const & data() const
    {
        return data_;
    }

    inline operator T()
    {
        return data_;
    }

    inline operator T&()
    {
        return data_;
    }

    inline operator T*()
    {
        return &data_;
    }

    inline operator const T&() const
    {
        return data_;
    }

    inline operator T () const
    {
        return data_;
    }
private:
    T              data_;
    time_t         time_;
} __attribute__ ((aligned (16)));
}
}

#endif // CSLIBS_MATH_STAMPED_HPP
