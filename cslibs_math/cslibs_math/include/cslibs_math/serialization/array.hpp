#ifndef CSLIBS_MATH_SERIALIZATION_ARRAY_HPP
#define CSLIBS_MATH_SERIALIZATION_ARRAY_HPP

#include <yaml-cpp/yaml.h>

#include <cslibs_math/serialization/binary.hpp>

namespace cslibs_math {
namespace serialization {
namespace array {
template <typename T, std::size_t Dim>
struct binary {
    inline static std::size_t read(std::ifstream      &in,
                                   std::array<T, Dim> &array)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            array[i] = io<T>::read(in);
        }
        return Dim * sizeof(T);
    }

    inline static void write(const std::array<T, Dim> &array,
                             std::ofstream &out)
    {
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            io<T>::write(array[i], out);
        }
    }
};
}
}
}

namespace YAML {
template <typename T, std::size_t Dim>
struct convert<std::array<T, Dim>>
{
    static Node encode(const std::array<T, Dim> &rhs)
    {
        Node n;

        for (std::size_t i = 0 ; i < Dim ; ++ i)
            n.push_back(rhs[i]);

        return n;
    }

    static bool decode(const Node& n, std::array<T, Dim> &rhs)
    {
        if (!n.IsSequence() || n.size() != Dim)
            return false;

        for (std::size_t i = 0 ; i < Dim ; ++ i)
            rhs[i] = n[i].as<T>();

        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_ARRAY_HPP
