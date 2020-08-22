#ifndef CSLIBS_MATH_2D_COVARIANCE_2D_HPP
#define CSLIBS_MATH_2D_COVARIANCE_2D_HPP

#include <cslibs_math/linear/matrix.hpp>

namespace cslibs_math_2d {
template <typename T>
using Covariance2 = cslibs_math::linear::Matrix<T, 3, 3>;
using Covariance2d = Covariance2<double>;
using Covariance2f = Covariance2<float>;
}

#endif // CSLIBS_MATH_2D_COVARIANCE_2D_HPP
