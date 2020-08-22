#ifndef CSLIBS_MATH_3D_POSE_3D_HPP
#define CSLIBS_MATH_3D_POSE_3D_HPP

#include <cslibs_math_3d/linear/transform.hpp>

namespace cslibs_math_3d {
template <typename T>
using Pose3 = Transform3<T>;
using Pose3d = Pose3<double>;
using Pose3f = Pose3<float>;
}

#endif // CSLIBS_MATH_3D_POSE_3D_HPP
