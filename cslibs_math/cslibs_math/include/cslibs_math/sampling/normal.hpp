#ifndef CSLIBS_MATH_NORMAL_SAMPLER_HPP
#define CSLIBS_MATH_NORMAL_SAMPLER_HPP

#include <memory>
#include <cslibs_math/random/random.hpp>
#include "traits.hpp"

namespace cslibs_math {
namespace sampling {
template<typename T, typename... Types>
class EIGEN_ALIGN16 Normal {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t    = Eigen::aligned_allocator<Normal>;
    using Ptr            = std::shared_ptr<Normal>;

    static const std::size_t Dimension = sizeof...(Types);
    static_assert(sizeof...(Types) > 0, "Constraint : Dimension > 0");
    static_assert(is_valid_type<Types...>::value, "Parameter list contains forbidden type!");

    using rng_t = cslibs_math::random::Normal<T,Dimension>;

    Normal() = delete;
    Normal(const Normal &other) = delete;

    Normal(const typename rng_t::sample_t &pose,
           const typename rng_t::matrix_t &covariance,
           const unsigned int seed = 0) :
        rng_(pose, covariance, seed)
    {
    }

    inline typename rng_t::sample_t get()
    {
        typename rng_t::sample_t sample = rng_.get();
        Arguments<Dimension, typename rng_t::sample_t, Types...>::normalize(sample);
        return sample;
    }

private:
    rng_t rng_;
}__attribute__ ((aligned (16)));
}
}

#endif /* CSLIBS_MATH_NORMAL_SAMPLER_HPP */
