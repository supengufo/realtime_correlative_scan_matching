#ifndef CSLIBS_MATH_3D_COVARIANCE_3D_HPP
#define CSLIBS_MATH_3D_COVARIANCE_3D_HPP

#include <cslibs_math/linear/matrix.hpp>

namespace cslibs_math_3d {
template <typename T>
using Covariance3 = cslibs_math::linear::Matrix<T, 6, 6>;
using Covariance3d = Covariance3<double>;
using Covariance3f = Covariance3<float>;
}

#endif // CSLIBS_MATH_3D_COVARIANCE_3D_HPP
