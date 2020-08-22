#ifndef CSLIBS_MATH_INDEX_HPP
#define CSLIBS_MATH_INDEX_HPP

#include <array>
#include <algorithm>
#include <assert.h>

namespace cslibs_math {
namespace common {

/**
 * @brief The Index struct is an extened version of the std::array<int, Dim>.
 *        It allows minimum, maximum calculation alongside primitive mathematic
 *        functions as addition and subtration.
 */
template<std::size_t _Nm>
struct Index : public std::array<int, _Nm>
{
    using base_t = std::array<int, _Nm>;

    /**
     * @brief Index default constructor.
     * @param i - constant value for each field in the array.
     */
    Index(const int i = 0)
    {
        base_t::fill(i);
    }

    /**
     * @brief Index constructor by std::arra<int, Dim>.
     * @param other - the other index
     */
    Index(const std::array<int, _Nm> &arr) :
                std::array<int, _Nm>(arr)
    {
    }

    void operator = (const int i)
    {
        base_t::fill(i);
    }

    void operator = (const base_t &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            base_t::_AT_Type::_S_ref(base_t::_M_elems, __n) =
                base_t::_AT_Type::_S_ref(other._M_elems, __n);
        }
    }

    /**
     * @brief max sets the maximum imperatively.
     * @param other - the other index
     */
    inline void max(const Index &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            base_t::_AT_Type::_S_ref(base_t::_M_elems, __n) =
                    std::max(base_t::_AT_Type::_S_ref(base_t::_M_elems, __n),
                             base_t::_AT_Type::_S_ref(other._M_elems, __n));
        }
    }

    /**
     * @brief min sets the minimum imperatively.
     * @param other - the other index
     */
    inline void min(const Index &other)
    {
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            base_t::_AT_Type::_S_ref(base_t::_M_elems, __n) =
                    std::min(base_t::_AT_Type::_S_ref(base_t::_M_elems, __n),
                             base_t::_AT_Type::_S_ref(other._M_elems, __n));
        }
    }

    /**
     * @brief max is the applicative version of building the maximum.
     * @param a - first index
     * @param b - second index
     * @return the maximum
     */
    static inline Index max(const Index &a,
                            const Index &b)
    {
        Index r;
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            base_t::_AT_Type::_S_ref(r._M_elems, __n) =
                    std::max(base_t::_AT_Type::_S_ref(a._M_elems, __n),
                             base_t::_AT_Type::_S_ref(b._M_elems, __n));
        }
        return r;
    }

    /**
     * @brief min is the applicative version of building the minimum.
     * @param a - first index
     * @param b - second index
     * @return the minimum
     */
    static inline Index min(const Index &a,
                            const Index &b)
    {
        Index r;
        for(std::size_t __n = 0 ; __n < _Nm ; ++__n) {
            base_t::_AT_Type::_S_ref(r._M_elems, __n) =
                    std::min(base_t::_AT_Type::_S_ref(a._M_elems, __n),
                             base_t::_AT_Type::_S_ref(b._M_elems, __n));
        }
        return r;
    }

};
}
}

//// primitive mathematical operators
template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator * (const cslibs_math::common::Index<_Nm> &__arr,
                                                   const int __s)
{
    cslibs_math::common::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __arr[i] * __s;
    return __res;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator - (const cslibs_math::common::Index<_Nm>& __one,
                                               const cslibs_math::common::Index<_Nm>& __two)
{
    cslibs_math::common::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __one[i] - __two[i];

    return __res;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator + (const cslibs_math::common::Index<_Nm>& __one,
                                               const cslibs_math::common::Index<_Nm>& __two)
{
    cslibs_math::common::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __one[i] + __two[i];

    return __res;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator - (const cslibs_math::common::Index<_Nm>& __idx,
                                               const int __scalar)
{
    cslibs_math::common::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __idx[i] - __scalar;

    return __res;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator + (const cslibs_math::common::Index<_Nm>& __idx,
                                               const int __scalar)
{
    cslibs_math::common::Index<_Nm> __res;
    for(std::size_t i = 0 ; i < _Nm; ++i)
        __res[i] = __idx[i] + __scalar;

    return __res;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator < (const cslibs_math::common::Index<_Nm> &__one,
                                                   const cslibs_math::common::Index<_Nm> &__two)
{
    bool less = true;
    for(std::size_t i = 0 ; i< _Nm; ++i)
        less &= __one[i] < __two[i];
    return less;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator > (const cslibs_math::common::Index<_Nm> &__one,
                                              const cslibs_math::common::Index<_Nm> &__two)
{
    bool greater = true;
    for(std::size_t i = 0 ; i< _Nm; ++i)
        greater &= __one[i] > __two[i];
    return greater;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator <= (const cslibs_math::common::Index<_Nm> &__one,
                                               const cslibs_math::common::Index<_Nm> &__two)
{
    bool less_equal = true;
    for(std::size_t i = 0 ; i< _Nm; ++i)
        less_equal &= __one[i] <= __two[i];
    return less_equal;
}

template<std::size_t _Nm>
inline cslibs_math::common::Index<_Nm> operator >= (const cslibs_math::common::Index<_Nm> &__one,
                                               const cslibs_math::common::Index<_Nm> &__two)
{
    bool greater_equal = true;
    for(std::size_t i = 0 ; i< _Nm; ++i)
        greater_equal &= __one[i] >= __two[i];
    return greater_equal;
}


#endif // INDEX_HPP
