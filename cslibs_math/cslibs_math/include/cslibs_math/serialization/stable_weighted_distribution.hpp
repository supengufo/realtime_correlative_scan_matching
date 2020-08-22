#ifndef CSLIBS_MATH_SERIALIZATION_STABLE_WEIGHTED_DISTRIBUTION_HPP
#define CSLIBS_MATH_SERIALIZATION_STABLE_WEIGHTED_DISTRIBUTION_HPP

#include <cslibs_math/statistics/stable_weighted_distribution.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <cslibs_math/serialization/binary.hpp>

namespace cslibs_math {
namespace serialization {
template <template <typename, std::size_t, std::size_t> class distribution_class_t,
          typename T, std::size_t Dim, std::size_t lambda_ratio_exponent>
struct binary;

template <typename T, std::size_t Dim, std::size_t lambda_ratio_exponent>
struct binary<cslibs_math::statistics::StableWeightedDistribution, T, Dim, lambda_ratio_exponent> {
    using distribution_t    = cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent>;
    using sample_t          = typename distribution_t::sample_t;
    using correlated_t      = typename distribution_t::covariance_t;

    static const std::size_t size = sizeof(std::size_t) + 2 * sizeof(T) + Dim * sizeof(T) + Dim * Dim * sizeof(T);

    inline static std::size_t read(std::ifstream  &in,
                                   distribution_t &distribution)
    {
        sample_t     mean;
        correlated_t corr;

        std::size_t sc = io<std::size_t>::read(in);
        T w            = io<T>::read(in);
        T w_sq         = io<T>::read(in);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            mean(i) = io<T>::read(in);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                corr(i,j) = io<T>::read(in);
            }
        }
        distribution = distribution_t(sc, w, w_sq, mean, corr);

        return size;
    }

    inline static void write(std::ofstream &out)
    {
        io<std::size_t>::write(0, out);
        io<T>::write(0, out);
        io<T>::write(0, out);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            io<T>::write(0.0, out);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                io<T>::write(0.0, out);
            }
        }
    }

    inline static void write(const distribution_t &distribution,
                             std::ofstream &out)
    {
        const sample_t      mean = distribution.getMean();
        const correlated_t  s    = distribution.getScatter();
        const std::size_t   sc   = distribution.getSampleCount();
        const T             w    = distribution.getWeight();
        const T             w_sq = distribution.getWeightSQ();

        io<std::size_t>::write(sc, out);
        io<T>::write(w, out);
        io<T>::write(w_sq, out);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            io<T>::write(mean(i), out);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                io<T>::write(s(i,j), out);
            }
        }
    }
};
}
}

namespace YAML {
template<typename T, std::size_t Dim, std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent>>
{
    using sample_t     = typename cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent>::sample_t;
    using covariance_t = typename cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent>::covariance_t;

    static Node encode(const cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getSampleCount());
        n.push_back(rhs.getWeight());
        n.push_back(rhs.getWeightSQ());

        sample_t mean = rhs.getMean();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            n.push_back(mean(i));

        covariance_t s = rhs.getScatter();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                n.push_back(s(i, j));

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent> &rhs)
    {
        if (!n.IsSequence() || n.size() != (3 + Dim + Dim * Dim))
            return false;

        std::size_t p  = 0;
        std::size_t sc = n[p++].as<std::size_t>();
        T w            = n[p++].as<T>();
        T w_sq         = n[p++].as<T>();

        sample_t mean(sample_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            mean(i) = n[p++].as<T>();

        covariance_t correlated(covariance_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                correlated(i, j) = n[p++].as<T>();

        rhs = cslibs_math::statistics::StableWeightedDistribution<T, Dim, lambda_ratio_exponent>(sc, w, w_sq, mean, correlated);
        return true;
    }
};

template<typename T, std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::StableWeightedDistribution<T, 1, lambda_ratio_exponent>>
{
    static Node encode(const cslibs_math::statistics::StableWeightedDistribution<T, 1, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getSampleCount());
        n.push_back(rhs.getWeight());
        n.push_back(rhs.getWeightSQ());
        n.push_back(rhs.getMean());
        n.push_back(rhs.getSquared());

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::StableWeightedDistribution<T, 1, lambda_ratio_exponent> &rhs)
    {
        if(!n.IsSequence() || n.size() != 5)
            return false;

        rhs = cslibs_math::statistics::StableWeightedDistribution<T, 1, lambda_ratio_exponent>(
                    n[0].as<std::size_t>(), n[1].as<T>(), n[2].as<T>(), n[3].as<T>(), n[4].as<T>());
        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_STABLE_WEIGHTED_DISTRIBUTION_HPP
