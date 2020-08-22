#ifndef CSLIBS_MATH_MEAN_HPP
#define CSLIBS_MATH_MEAN_HPP

#include <eigen3/Eigen/Core>

namespace cslibs_math {
namespace statistics {
template<typename T, std::size_t Dim>
class EIGEN_ALIGN16 Mean
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Mean<T,Dim>>;

    using Ptr         = std::shared_ptr<Mean<T,Dim>>;
    using sample_t    = Eigen::Matrix<T, Dim, 1>;

    inline Mean() :
        mean_(Eigen::Matrix<T, Dim, 1>::Zero()),
        n_(0)
    {
    }

    inline Mean(const Mean &other) :
        mean_(other.mean_),
        n_(other.n_)
    {
    }

    inline Mean(Mean &&other) :
        mean_(std::move(other.mean_)),
        n_(other.n_)
    {
    }

    inline void add(const sample_t &sample)
    {
        const std::size_t _n = n_+1;
        mean_ = (mean_ * static_cast<T>(n_) + sample) / static_cast<T>(_n);
        n_    = _n;
    }

    inline sample_t get() const
    {
        return mean_;
    }

    inline std::size_t getN() const
    {
        return n_;
    }

private:
    sample_t    mean_;
    std::size_t n_;
};

template<typename T>
class EIGEN_ALIGN16 Mean<T, 1>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Mean<T,1>>;

    Mean() :
        mean_(T()),
        n_(0)
    {
    }

    inline void add(const T &sample)
    {
        const std::size_t _n = n_+1;
        mean_ = (mean_ * static_cast<T>(n_) + sample) / static_cast<T>(_n);
        n_    = _n;
    }

    inline T get() const
    {
        return mean_;
    }

    inline std::size_t getN() const
    {
        return n_;
    }

private:
    T mean_;
    std::size_t n_;
};
}
}

#endif // CSLIBS_MATH_MEAN_HPP
