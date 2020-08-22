#ifndef CSLIBS_MATH_ARRAY_HPP
#define CSLIBS_MATH_ARRAY_HPP

#include <array>
#include <iostream>
#include <unordered_set>

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator - (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] - __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator - (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] - s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator + (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] + __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator + (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] + s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator * (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] * __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator * (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] *= s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator / (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] / __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator / (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] /= s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::ostream & operator << (std::ostream &__out, const std::array<_Tp, _Nm> &__arr)
{
    if(_Nm != 0ul) {
        __out << "[";
        for(std::size_t i = 0 ; i < _Nm-1 ; ++i) {
            __out << __arr[i] << ",";
        }
        __out << __arr.back() << "]";
    } else {
        __out << "[]";
    }
    return __out;
}

namespace std {
template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> min(const std::array<_Tp, _Nm> &__one,
                         const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::min(__one[i], __two[i]);
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> max(const std::array<_Tp, _Nm> &__one,
                         const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::max(__one[i], __two[i]);
    return arr;
}
template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> abs(const std::array<_Tp, _Nm> &__one)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::abs(__one[i]);
    return arr;
}
template<typename _Tp, std::size_t _Nm>
std::array<int, _Nm> compare(const std::array<_Tp, _Nm> &__one,
                             const std::array<_Tp, _Nm> &__two)
{
    std::array<int, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] < __two[i] ? 1 : -1;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<int, _Nm> compare(const std::array<_Tp, _Nm> &__one,
                             const _Tp s)
{
    std::array<int, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] < s ? 1 : -1;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> ceil(const std::array<_Tp, _Nm> &__one)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::ceil(__one[i]);
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> floor(const std::array<_Tp, _Nm> &__one)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::floor(__one[i]);
    return arr;
}

template<typename T, size_t N>
struct hash<array<T, N> >
{
    typedef array<T, N> argument_type;
    typedef size_t result_type;

    result_type operator()(const argument_type& a) const
    {
        hash<T> hasher;
        result_type h = 0;
        for (result_type i = 0; i < N; ++i)
        {
            h = h * 31 + hasher(a[i]);
        }
        return h;
    }
};
}

namespace cslibs_math {
namespace common {
template<typename _T, typename _Tp, std::size_t _Nm>
std::array<_T, _Nm> cast(const std::array<_Tp, _Nm> &__one)
{
    std::array<_T, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = static_cast<_T>(__one[i]);
    return arr;
}
}
}

#endif // CSLIBS_MATH_ARRAY_HPP
