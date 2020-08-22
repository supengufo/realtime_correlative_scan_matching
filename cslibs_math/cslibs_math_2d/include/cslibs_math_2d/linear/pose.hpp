#ifndef CSLIBS_MATH_2D_POSE_2D_HPP
#define CSLIBS_MATH_2D_POSE_2D_HPP

#include <cslibs_math_2d/linear/transform.hpp>

namespace cslibs_math_2d {
template <typename T>
using Pose2 = Transform2<T>;
using Pose2d = Pose2<double>;
using Pose2f = Pose2<float>;
}

#endif // CSLIBS_MATH_2D_POSE_2D_HPP
