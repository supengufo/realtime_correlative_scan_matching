#ifndef CSLIBS_MATH_ROS_TF_CONVERSION_2D_HPP
#define CSLIBS_MATH_ROS_TF_CONVERSION_2D_HPP

#include <tf/tf.h>
#include <cslibs_math_2d/linear/covariance.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

namespace cslibs_math_ros {
namespace tf {
namespace conversion_2d {
template <typename T>
inline cslibs_math_2d::Vector2<T> from(const ::tf::Vector3 &v)
{
    return cslibs_math_2d::Vector2<T>(v.x(), v.y());
}

template <typename T>
inline cslibs_math_2d::Transform2<T> from(const ::tf::Transform &t)
{
    return cslibs_math_2d::Transform2<T>(from<T>(t.getOrigin()),
                                          ::tf::getYaw(t.getRotation()));
}

template <typename T>
inline ::tf::Vector3 from(const cslibs_math_2d::Vector2<T> &v)
{
    return ::tf::Vector3(v(0), v(1), T());
}

template <typename T>
inline ::tf::Transform from(const cslibs_math_2d::Transform2<T> &t)
{
    return ::tf::Transform(::tf::createQuaternionFromYaw(t.yaw()),
                          from(t.translation()));
}

template <typename T>
inline void from(const std::vector<::tf::Transform> &src,
                 std::vector<cslibs_math_2d::Transform2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const ::tf::Transform &t){return from<T>(t);});
}

template <typename T>
inline void from(const std::vector<::tf::Vector3> &src,
                 std::vector<cslibs_math_2d::Vector2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const ::tf::Vector3 &v){return from<T>(v);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_2d::Transform2<T>> &src,
                 std::vector<::tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Transform2<T> &t){return from(t);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_2d::Vector2<T>> &src,
                 std::vector<::tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Vector2<T> &v){return from(v);});
}
}
}
}

#endif // CSLIBS_MATH_ROS_TF_CONVERSION_2D_HPP
