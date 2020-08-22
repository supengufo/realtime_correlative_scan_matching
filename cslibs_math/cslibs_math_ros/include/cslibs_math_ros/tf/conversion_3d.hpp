#ifndef CSLIBS_MATH_ROS_TF_CONVERSION_3D_HPP
#define CSLIBS_MATH_ROS_TF_CONVERSION_3D_HPP

#include <tf/tf.h>

#include <cslibs_math_3d/linear/covariance.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

namespace cslibs_math_ros {
namespace tf {
namespace conversion_3d {
template <typename T>
inline cslibs_math_3d::Vector3<T> from(const ::tf::Vector3 &v)
{
    return cslibs_math_3d::Vector3<T>(v.x(), v.y(), v.z());
}

template <typename T>
inline ::tf::Vector3 from(const cslibs_math_3d::Vector3<T> &v)
{
    return ::tf::Vector3(v(0), v(1), v(2));
}

template <typename T>
inline ::tf::Quaternion from(const cslibs_math_3d::Quaternion<T> &q)
{
    return ::tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

template <typename T>
inline cslibs_math_3d::Quaternion<T> from(const ::tf::Quaternion &q)
{
    return cslibs_math_3d::Quaternion<T>(q.x(), q.y(), q.z(), q.w());
}

template <typename T>
inline cslibs_math_3d::Transform3<T> from(const ::tf::Transform &t)
{
    return cslibs_math_3d::Transform3<T>(from<T>(t.getOrigin()),
                                          from<T>(t.getRotation()));
}

template <typename T>
inline ::tf::Transform from(const cslibs_math_3d::Transform3<T> &t)
{
    return ::tf::Transform(from(t.rotation()),
                           from(t.translation()));
}

template <typename T>
inline void from(const std::vector<::tf::Transform> &src,
                 std::vector<cslibs_math_3d::Transform3<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const ::tf::Transform &t){return from<T>(t);});
}

template <typename T>
inline void from(const std::vector<::tf::Vector3> &src,
                 std::vector<cslibs_math_3d::Transform3<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const ::tf::Vector3 &v){return from<T>(v);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_3d::Transform3<T>> &src,
                 std::vector<::tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_3d::Transform3<T> &t){return from(t);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_3d::Vector3<T>> &src,
                 std::vector<::tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_3d::Vector3<T> &v){return from(v);});
}
}
}
}

#endif // CSLIBS_MATH_ROS_TF_CONVERSION_3D_HPP
