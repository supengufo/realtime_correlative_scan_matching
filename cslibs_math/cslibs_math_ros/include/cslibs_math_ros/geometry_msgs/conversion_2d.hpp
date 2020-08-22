#ifndef CSLIBS_MATH_ROS_GEOMETRY_MSGS_CONVERSION_2D_HPP
#define CSLIBS_MATH_ROS_GEOMETRY_MSGS_CONVERSION_2D_HPP

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>

#include <cslibs_math_2d/linear/covariance.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

namespace cslibs_math_ros {
namespace geometry_msgs {
namespace conversion_2d {
template <typename T>
inline cslibs_math_2d::Vector2<T> from(const ::geometry_msgs::Point &p)
{
    return cslibs_math_2d::Vector2<T>(p.x, p.y);
}

template <typename T>
inline cslibs_math_2d::Transform2<T> from(const ::geometry_msgs::Pose &p)
{
    return cslibs_math_2d::Transform2<T>(from<T>(p.position),
                                          ::tf::getYaw(p.orientation));
}

template <typename T>
inline ::geometry_msgs::Point from(const cslibs_math_2d::Vector2<T> &v)
{
    ::geometry_msgs::Point p;
    p.x = v(0);
    p.y = v(1);
    p.z = 0.0;
    return p;
}

template <typename T>
inline ::geometry_msgs::Pose from(const cslibs_math_2d::Transform2<T> &t)
{
    ::geometry_msgs::Pose p;
    p.orientation = ::tf::createQuaternionMsgFromYaw(t.yaw());
    p.position = from(t.translation());
    return p;
}

template <typename T>
inline void from(const std::vector<::geometry_msgs::Pose> &src,
                 std::vector<cslibs_math_2d::Transform2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const ::geometry_msgs::Pose &p){return from<T>(p);});
}

template <typename T>
inline void from(const std::vector<::geometry_msgs::Point> &src,
                 std::vector<cslibs_math_2d::Vector2<T>> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const ::geometry_msgs::Point &p){return from<T>(p);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_2d::Transform2<T>> &src,
                 std::vector<::geometry_msgs::Pose> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Transform2<T> &t){return from(t);});
}

template <typename T>
inline void from(const std::vector<cslibs_math_2d::Vector2<T>> &src,
                 std::vector<::geometry_msgs::Point> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Vector2<T> &v){return from(v);});
}
}
}
}

#endif // CSLIBS_MATH_ROS_GEOMETRY_MSGS_CONVERSION_2D_HPP
