#ifndef CSLIBS_MATH_ANGLE_HPP
#define CSLIBS_MATH_ANGLE_HPP

#include <cmath>
#include <complex>

namespace cslibs_math {
namespace common {
namespace angle {
/**
 * @brief normalize and between -pi and pi
 * @param angle
 * @return
 */
template <typename T>
inline T normalize(const T angle)
{
    static const T _2_M_PI = 2.0 * static_cast<T>(M_PI);
    static const T _1_2_M_PI = 1.0 / _2_M_PI;
    return angle - _2_M_PI * floor((angle + M_PI) * _1_2_M_PI);
}

/**
 * @brief normalize2Pi normalizes angles within interval from [0.0, 2 * PI)
 * @param angle - angle to normalize
 * @return
 */
template <typename T>
inline T normalize2Pi(const T angle)
{
    static const T _2_M_PI = 2.0 * static_cast<T>(M_PI);
    static const T _1_2_M_PI = 1.0 / _2_M_PI;
    return angle - _2_M_PI * floor(angle * _1_2_M_PI);
}

/**
 * @brief difference calculates the normalized angle difference.
 * @param a - first angle in term
 * @param b - second angle in term
 * @return
 */
template <typename T>
inline T difference(T a, T b)
{
    static const T _2_M_PI = 2.0 * static_cast<T>(M_PI);
    auto norm = [](T v) { return std::atan2(std::sin(v), std::cos(v)); };
    a = norm(a);
    b = norm(b);

    T d1 = a - b;
    T d2 = (_2_M_PI - std::fabs(d1)) * (d1 > 0 ? -1 : 1);
    return std::fabs(d1) < std::fabs(d2) ? d1 : d2;
}

/**
 * @brief toRad converts angles given in degree to radien.
 * @param deg   - the angle to convert
 * @return      - angle in radian
 */
template <typename T>
inline T toRad(const T deg)
{
    static const T R_T_A = static_cast<T>(M_PI / 180.0);       /// DEG to RAD
    return deg * R_T_A;
}

/**
 * @brief fromRad converts angle from radien to degree.
 * @param rad   - the angle in radian
 * @return      - the angle in degree
 */
template <typename T>
inline T fromRad(const T rad)
{
    static const T A_T_R = static_cast<T>(M_1_PI * 180.0);      /// RAD ot DEG
    return rad * A_T_R;
}

/**
 * @brief toComplex converts an angle given in radian to its complex
 *        representation.
 * @param rad   - the angle in radian
 * @return      - the angle in its complex representation
 */
template <typename T>
inline std::complex<T> toComplex(const T rad)
{
    return std::complex<T>(std::cos(rad), std::sin(rad));
}
/**
 * @brief fromComplex converts an angle from its complex representation to
 *        radian, so it is geometrically interpretable again.
 * @param complex - the complex representation of the angle
 * @return        - the angle in radian
 */
template <typename T>
inline T fromComplex(const std::complex<T> &complex)
{
    return std::atan2(complex.imag(), complex.real());
}
}
}
}

#endif // CSLIBS_MATH_ANGLE_HPP
