#ifndef CSLIBS_MATH_EIGEN_HPP
#define CSLIBS_MATH_EIGEN_HPP

#include <eigen3/Eigen/Core>
#include <iostream>

namespace cslibs_math {
namespace linear {
namespace eigen {
namespace impl {
template<std::size_t index, typename Array>
struct isnormal
{
    static bool apply (const Array &arr)
    {
        const static constexpr unsigned int rows = Array::RowsAtCompileTime;
        const static constexpr unsigned int cols = Array::ColsAtCompileTime;
        static constexpr unsigned int r = rows - 1ul - index / cols;
        static constexpr unsigned int c = cols - 1ul - index % cols;
        return std::isnormal(arr(r,c)) && isnormal<index-1, Array>::apply(arr);
    }
};

template<typename Array>
struct isnormal<0, Array>
{
    static bool apply (const Array &arr)
    {
        return std::isnormal(arr(Array::RowsAtCompileTime - 1, Array::ColsAtCompileTime-1));
    }
};
}

template<typename Array, typename T>
inline void assign(Array &arr, const T t)
{
    arr(Array::RowsAtCompileTime - 1, Array::ColsAtCompileTime-1) = t;
}

template<typename Array, typename T, typename... Ts>
inline void assign(Array &arr, const T t, const Ts... ts)
{
    const static constexpr unsigned int rows = Array::RowsAtCompileTime;
    const static constexpr unsigned int cols = Array::ColsAtCompileTime;

    static constexpr unsigned int i = sizeof...(ts);
    static constexpr unsigned int r = rows - 1u - i / cols;
    static constexpr unsigned int c = cols - 1u - i % cols;
    arr(r,c) = t;
    assign(arr, ts...);
}
template<typename Array, typename... Ts>
inline Array create(const Ts... ts)
{
    Array arr;
    assign(arr, ts...);
    return arr;
}


template<typename Array>
inline bool isnormal(const Array &arr)
{
    const static constexpr unsigned int rows = Array::RowsAtCompileTime;
    const static constexpr unsigned int cols = Array::ColsAtCompileTime;
    return impl::isnormal<rows * cols - 1, Array>::apply(arr);
}
}
}
}

#endif // CSLIBS_MATH_EIGEN_HPP
