#ifndef CSLIBS_MATH_STABLE_StableDistribution_HPP
#define CSLIBS_MATH_STABLE_StableDistribution_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <cslibs_math/statistics/limit_eigen_values.hpp>
#include <cslibs_math/common/sqrt.hpp>

namespace cslibs_math {
namespace statistics {
template<typename T, std::size_t Dim, std::size_t lambda_ratio_exponent = 0>
class EIGEN_ALIGN16 StableDistribution {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t         = Eigen::aligned_allocator<StableDistribution<T, Dim, lambda_ratio_exponent>>;

    using Ptr                 = std::shared_ptr<StableDistribution<T, Dim, lambda_ratio_exponent>>;
    using sample_t            = Eigen::Matrix<T, Dim, 1>;
    using sample_transposed_t = Eigen::Matrix<T, 1, Dim>;
    using covariance_t        = Eigen::Matrix<T, Dim, Dim>;
    using eigen_values_t      = Eigen::Matrix<T, Dim, 1>;
    using eigen_vectors_t     = Eigen::Matrix<T, Dim, Dim>;

    /// @attention this may cause problems with debug compiles
    static constexpr T sqrt_2_M_PI = static_cast<T>(cslibs_math::common::sqrt(2.0 * M_PI));

    inline StableDistribution() :
        mean_(sample_t::Zero()),
        scatter_(covariance_t::Zero()),
        n_(0),
        covariance_(covariance_t::Zero()),
        information_matrix_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(T()),  // zero initialization
        dirty_(false),
        dirty_eigenvalues_(false)
    {
    }

    inline StableDistribution(
            std::size_t  n,
            sample_t     mean,
            covariance_t scatter) :
        mean_(mean),
        scatter_(scatter),
        n_(n),
        covariance_(covariance_t::Zero()),
        information_matrix_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(T()),  // zero initialization
        dirty_(true),
        dirty_eigenvalues_(true)
    {
    }

    inline StableDistribution(const StableDistribution &other) :
        mean_(other.mean_),
        scatter_(other.scatter_),
        n_(other.n_),
        covariance_(other.covariance_),
        information_matrix_(other.information_matrix_),
        eigen_values_(other.eigen_values_),
        eigen_vectors_(other.eigen_vectors_),
        determinant_(other.determinant_),
        dirty_(other.dirty_),
        dirty_eigenvalues_(other.dirty_eigenvalues_)
    {
    }

    inline StableDistribution& operator=(const StableDistribution &other)
    {
        mean_                 = other.mean_;
        scatter_              = other.scatter_;
        n_                    = other.n_;

        covariance_           = other.covariance_;
        information_matrix_   = other.information_matrix_;
        eigen_values_         = other.eigen_values_;
        eigen_vectors_        = other.eigen_vectors_;
        determinant_          = other.determinant_;

        dirty_                = other.dirty_;
        dirty_eigenvalues_    = other.dirty_eigenvalues_;
        return *this;
    }

    inline StableDistribution(StableDistribution &&other)
    {
        mean_                 = std::move(other.mean_);
        scatter_              = std::move(other.scatter_);
        n_                    = other.n_;

        covariance_           = std::move(other.covariance_);
        information_matrix_   = std::move(other.information_matrix_);
        eigen_values_         = std::move(other.eigen_values_);
        eigen_vectors_        = std::move(other.eigen_vectors_);
        determinant_          = other.determinant_;

        dirty_                = other.dirty_;
        dirty_eigenvalues_    = other.dirty_eigenvalues_;
    }

    inline StableDistribution& operator=(StableDistribution &&other)
    {
        mean_                 = std::move(other.mean_);
        scatter_              = std::move(other.scatter_);
        n_                    = other.n_;

        covariance_           = std::move(other.covariance_);
        information_matrix_   = std::move(other.information_matrix_);
        eigen_values_         = std::move(other.eigen_values_);
        eigen_vectors_        = std::move(other.eigen_vectors_);
        determinant_          = other.determinant_;

        dirty_                = other.dirty_;
        dirty_eigenvalues_    = other.dirty_eigenvalues_;
        return *this;
    }

    inline void reset()
    {
        mean_               = sample_t::Zero();
        scatter_            = covariance_t::Zero();
        n_                  = 0;

        covariance_         = covariance_t::Zero();
        information_matrix_ = covariance_t::Zero();
        eigen_vectors_      = eigen_vectors_t::Zero();
        eigen_values_       = eigen_values_t::Zero();
        determinant_        = T();

        dirty_              = true;
        dirty_eigenvalues_  = true;
    }

    /// Modification
    inline void add(const sample_t &p)
    {
        const sample_t _mean = mean_;
        const std::size_t _n = n_+1;
        mean_     = (mean_ * static_cast<T>(n_) + p) / static_cast<T>(_n);
        scatter_ += (p - _mean) * (p - mean_).transpose();
        n_        = _n;
        dirty_    = true;
        dirty_eigenvalues_ = true;
    }

    inline StableDistribution& operator += (const sample_t &p)
    {
        add(p);
        return *this;
    }

    inline StableDistribution& operator += (const StableDistribution &other)
    {
        const std::size_t _n = n_ + other.n_;
        const auto dmean = mean_ - other.mean_;
        mean_     = (mean_ * static_cast<T>(n_) + other.mean_ * static_cast<T>(other.n_)) / static_cast<T>(_n);
        scatter_ += other.scatter_ + static_cast<T>(n_ * other.n_)/static_cast<T>(_n) * dmean * dmean.transpose();
        n_        = _n;
        dirty_    = true;
        dirty_eigenvalues_ = true;
        return *this;
    }

    inline bool valid() const
    {
        return n_ > Dim;
    }

    inline std::size_t getN() const
    {
        return n_;
    }

    inline sample_t getMean() const
    {
        return mean_;
    }

    inline void getMean(sample_t &_mean) const
    {
        _mean = sample_t(mean_);
    }

    inline covariance_t getScatter() const
    {
        return scatter_;
    }

    inline covariance_t getCovariance() const
    {
        auto update_return_covariance = [this](){
            update(); return covariance_;
        };
        return (dirty_ && valid()) ? update_return_covariance() : covariance_;
    }

    inline void getScatter(covariance_t &s) const
    {
        s = covariance_t(scatter_);
    }

    inline void getCovariance(covariance_t &covariance) const
    {
        auto update_return_covariance = [this](){
            update(); return covariance_t(covariance_);
        };
        covariance = (dirty_ && valid()) ? update_return_covariance() : covariance_t(covariance_);
    }

    inline covariance_t getInformationMatrix() const
    {
        auto update_return_information = [this](){
            update(); return information_matrix_;
        };
        return (dirty_ && valid()) ? update_return_information() : information_matrix_;
    }

    inline void getInformationMatrix(covariance_t &information_matrix) const
    {
        auto update_return_information = [this](){
            update(); return covariance_t(information_matrix_);
        };
        information_matrix = (dirty_ && valid()) ? update_return_information() : covariance_t(information_matrix_);
    }

    inline eigen_values_t getEigenValues(const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            updateEigenvalues(); return abs ? eigen_values_t(eigen_values_.cwiseAbs()) : eigen_values_;
        };
        return (dirty_eigenvalues_ && valid()) ?  update_return_eigen() : (abs ? eigen_values_t(eigen_values_.cwiseAbs()) : eigen_values_);
    }

    inline void getEigenValues(eigen_values_t &eigen_values,
                               const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            updateEigenvalues(); return abs ? eigen_values_t(eigen_values_.cwiseAbs()) : eigen_values_t(eigen_values_);
        };
        eigen_values = (dirty_eigenvalues_ && valid()) ? update_return_eigen() : (abs ? eigen_values_t(eigen_values_.cwiseAbs()) : eigen_values_t(eigen_values_));
    }

    inline eigen_vectors_t getEigenVectors() const
    {
        auto update_return_eigen = [this]() {
            updateEigenvalues(); return eigen_vectors_;
        };
        return (dirty_eigenvalues_ && valid()) ? update_return_eigen() : eigen_vectors_;
    }

    inline void getEigenVectors(eigen_vectors_t &eigen_vectors) const
    {
        auto update_return_eigen = [this]() {
            updateEigenvalues(); return eigen_vectors_t(eigen_vectors_);
        };
        eigen_vectors = (dirty_eigenvalues_ && valid()) ? update_return_eigen() : eigen_vectors_t(eigen_vectors_);
    }

    /// Evaluation
    inline T denominator() const
    {
        auto update_return = [this](){
            if (dirty_) update();
            return 1.0 / (determinant_ * sqrt_2_M_PI);
        };
        return valid() ? update_return() : T(); // T() = zero
    }

    inline T sample(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if (dirty_) update();
            const auto& q = p - mean_;
            const auto& exponent = - 0.5 * q.transpose() * information_matrix_ * q;
            const auto& denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return valid() ? update_sample() : T(); // T() = zero
    }

    inline T sample(const sample_t &p,
                    sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if (dirty_) update();
            q = p - mean_;
            const auto& exponent = - 0.5 * q.transpose() * information_matrix_ * q;
            const auto& denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return valid() ? update_sample() : T(); // T() = zero
    }

    inline T sampleMean() const
    {
        return sample(getMean());
    }

    inline T sampleNonNormalized(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if (dirty_) update();
            const auto& q = p - mean_;
            const T& exponent = - 0.5 * q.transpose() * information_matrix_ * q;
            return std::exp(exponent);
        };
        return valid() ? update_sample() : T(); // T() = zero
    }

    inline T sampleNonNormalized(const sample_t &p,
                                 sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if (dirty_) update();
            q = p - mean_;
            const T exponent = - 0.5 * q.transpose() * information_matrix_ * q;
            return std::exp(exponent);
        };
        return valid() ? update_sample() : T(); // T() = zero
    }

    inline T sampleNonNormalizedMean() const
    {
        return sampleNonNormalized(getMean());
    }

    inline void merge(const StableDistribution&)
    {
    }

private:
    sample_t                     mean_;
    covariance_t                 scatter_;
    std::size_t                  n_;

    mutable covariance_t         covariance_;
    mutable covariance_t         information_matrix_;
    mutable eigen_values_t       eigen_values_;
    mutable eigen_vectors_t      eigen_vectors_;
    mutable T                    determinant_;

    mutable bool                 dirty_;
    mutable bool                 dirty_eigenvalues_;

    inline void update() const
    {
        const T scale = T(1) / static_cast<T>(n_ - 1);
        covariance_ = scale * scatter_;
        /*
        for (std::size_t i = 0 ; i < Dim ; ++i) {
            for (std::size_t j = i ; j < Dim ; ++j) {
                covariance_(i, j) = scale * scatter_(i,j);
                covariance_(j, i) = covariance_(i, j);
            }
        }*/

        LimitEigenValues<T, Dim, lambda_ratio_exponent>::apply(covariance_);

        information_matrix_ = covariance_.inverse();
        determinant_        = covariance_.determinant();

        dirty_              = false;
    }

    inline void updateEigenvalues() const
    {
        if (dirty_)
            update();

        Eigen::EigenSolver<covariance_t> solver;
        solver.compute(covariance_);
        eigen_vectors_ = solver.eigenvectors().real();
        eigen_values_  = solver.eigenvalues().real();

        dirty_eigenvalues_ = false;
    }
};

template<typename T, std::size_t lambda_ratio_exponent>
class StableDistribution<T, 1, lambda_ratio_exponent>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<StableDistribution<T, 1, lambda_ratio_exponent>>;
    using Ptr         = std::shared_ptr<StableDistribution<T, 1, lambda_ratio_exponent>>;

    static constexpr T sqrt_2_M_PI = cslibs_math::common::sqrt(2.0 * M_PI);

    inline StableDistribution() :
        mean_(T()),
        scatter_(T()),
        n_(0),
        variance_(T()),
        standard_deviation_(T()),
        dirty_(false)
    {
    }

    inline StableDistribution(
            std::size_t n,
            T           mean,
            T           scatter) :
        mean_(mean),
        scatter_(scatter),
        n_(n),
        variance_(T()),
        standard_deviation_(T()),
        dirty_(true)
    {
    }

    inline StableDistribution(const StableDistribution &other) = default;
    inline StableDistribution(StableDistribution &&other) = default;
    inline StableDistribution& operator=(const StableDistribution &other) = default;

    inline void reset()
    {
        mean_               = T();
        scatter_            = T();
        n_                  = 0;
        variance_           = T();
        standard_deviation_ = T();
        dirty_              = false;
    }

    inline void add(const T s)
    {
        const T _mean = mean_;
        const std::size_t _n = n_+1;
        mean_     = (mean_ * static_cast<T>(n_) + s) / static_cast<T>(_n);
        scatter_ += (s - _mean) * (s - mean_);
        n_        = _n;
        dirty_    = true;
    }

    inline StableDistribution & operator += (const T s)
    {
        add(s);
        return *this;
    }

    inline StableDistribution & operator += (const StableDistribution &other)
    {
        const std::size_t _n = n_ + other.n_;
        const auto dmean = mean_ - other.mean_;
        mean_     = (mean_ * static_cast<T>(n_) + other.mean_ * static_cast<T>(other.n_)) / static_cast<T>(_n);
        scatter_ += other.scatter_ + static_cast<T>(n_ * other.n_)/static_cast<T>(_n) * dmean * dmean;
        n_        = _n;
        dirty_    = true;
        return *this;
    }

    inline bool valid() const
    {
        return n_ > 1;
    }

    inline std::size_t getN() const
    {
        return n_;
    }

    inline T getMean() const
    {
        return mean_;
    }

    inline T getScatter() const
    {
        return scatter_;
    }

    inline T getVariance() const
    {
        auto update_return_variance = [this]() {
            update(); return variance_;
        };
        return (dirty_ && valid()) ? update_return_variance() : variance_;
    }

    inline T getStandardDeviation() const
    {
        auto update_return_standard_deviation = [this]() {
            update(); return standard_deviation_;
        };
        return (dirty_ && valid()) ? update_return_standard_deviation() : standard_deviation_;
    }

    inline T sample(const T s) const
    {
        auto update_sample = [this, &s]() {
            if (dirty_) update();
            const T d = 2.0 * variance_;
            const T x = s - mean_;
            return std::exp(-0.5 * x * x / d) / (sqrt_2_M_PI * standard_deviation_);
        };
        return valid() ? update_sample() : T();
    }

    inline T sampleNonNormalized(const T s) const
    {
        auto update_sample = [this, &s]() {
            if (dirty_) update();
            const T d = 2.0 * variance_;
            const T x = s - mean_;
            return std::exp(-0.5 * x * x / d);
        };
        return valid() ? update_sample() : T();
    }

    inline void merge(const StableDistribution&)
    {
    }

private:
    T               mean_;
    T               scatter_;
    std::size_t     n_;

    mutable T       variance_;
    mutable T       standard_deviation_;

    mutable bool    dirty_;

    inline void update() const
    {
        variance_ = scatter_ / static_cast<T>(n_ - 1);
        standard_deviation_ = std::sqrt(variance_);
        dirty_ = false;
    }
};
}
}

template<typename T, std::size_t D, std::size_t L>
std::ostream & operator << (std::ostream &out, const cslibs_math::statistics::StableDistribution<T,D,L> &d)
{
    out << d.getMean() << "\n";
    out << d.getCovariance() << "\n";
    out << d.getN() << "\n";
    return out;
}

template<typename T, std::size_t L>
std::ostream & operator << (std::ostream &out, const cslibs_math::statistics::StableDistribution<T,1,L> &d)
{
    out << d.getMean() << "\n";
    out << d.getVariance() << "\n";
    out << d.getN() << "\n";
    return out;
}

#endif // CSLIBS_MATH_STABLE_StableDistribution_HPP
