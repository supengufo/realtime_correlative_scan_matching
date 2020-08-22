#ifndef CSLIBS_MATH_SERIALIZATION_BINARY_HPP
#define CSLIBS_MATH_SERIALIZATION_BINARY_HPP

#include <fstream>

namespace cslibs_math {
namespace serialization {
template<typename T>
struct io
{
    union Buffer{
        char b[sizeof(T)];
        T    v;
    };

    inline static T read(std::ifstream &in)
    {
        Buffer b;
        for(unsigned int i = 0 ; i < sizeof(T) ; ++i)
            b.b[i] = 0;
        in.read((char*)(&(b.b)), sizeof(T));
        return b.v;
    }

    inline static void write(const T v,
                             std::ofstream &out)
    {
        Buffer b;
        for(unsigned int i = 0 ; i < sizeof(T) ; ++i)
            b.b[i] = 0;

        b.v = v;
        out.write((char*)(&b.b), sizeof(T));
    }
};
}
}
#endif // CSLIBS_MATH_SERIALIZATION_BINARY_HPPs
