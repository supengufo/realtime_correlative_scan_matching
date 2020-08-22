#ifndef CSLIBS_MATH_SERIALIZATION_MATRIX_HPP
#define CSLIBS_MATH_SERIALIZATION_MATRIX_HPP

#include <cslibs_math/linear/matrix.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<typename T, std::size_t N, std::size_t M>
struct convert<cslibs_math::linear::Matrix<T, N, M>>
{
    static Node encode(const cslibs_math::linear::Matrix<T, N, M> &rhs)
    {
        Node n;
        for(std::size_t i = 0 ; i < N ; ++i) {
            for(std::size_t j = 0 ; j < M ; ++j) {
                n.push_back(rhs(i,j));
            }
        }
        return n;
    }
    static bool decode(const Node& n, cslibs_math::linear::Matrix<T, N, M> &rhs)
    {
        if(!n.IsSequence() || n.size() != N * M)
            return false;

        std::size_t p = 0;
        for(std::size_t i = 0 ; i < N ; ++i) {
            for(std::size_t j = 0 ; j < M ; ++j) {
                rhs(i,j) = n[p].as<T>();
                ++p;
            }
        }
        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_MATRIX_HPP
