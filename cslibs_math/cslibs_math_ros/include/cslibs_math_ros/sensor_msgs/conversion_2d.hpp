#ifndef CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_2D_HPP
#define CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_2D_HPP

#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/polar_pointcloud.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <cslibs_time/time_frame.hpp>

#include <cmath>
#include <sensor_msgs/LaserScan.h>

namespace cslibs_math_ros {
namespace sensor_msgs {
namespace conversion_2d {
using interval_t = std::array<float, 2>;

inline cslibs_time::TimeFrame from(const ::sensor_msgs::LaserScan::ConstPtr &src)
{
    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(static_cast<double>(src->time_increment)) * static_cast<double>(src->ranges.size());
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(static_cast<double>(src->scan_time));
    }
    return cslibs_time::TimeFrame(start_stamp.toNSec(),
                                  start_stamp.toNSec() + static_cast<uint64_t>(delta_stamp.toNSec()));
}

inline cslibs_time::TimeFrame from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                                   const interval_t &angular_interval)
{
    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(static_cast<double>(src->time_increment)) * static_cast<double>(src->ranges.size());
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(static_cast<double>(src->scan_time));
    }

    const double fov             = static_cast<double>(src->angle_max - src->angle_min);
    const double start_off_ratio = static_cast<double>(std::max(src->angle_min, angular_interval[0]) - src->angle_min) / fov;
    const double end_off_ratio   = static_cast<double>(std::min(src->angle_max, angular_interval[1]) - src->angle_min) / fov;

    const int64_t start_time = static_cast<int64_t>(std::floor(start_stamp.toNSec() + delta_stamp.toNSec() * start_off_ratio + 0.5));
    const int64_t end_time   = static_cast<int64_t>(std::floor(start_stamp.toNSec() + delta_stamp.toNSec() * end_off_ratio  + 0.5));

    return cslibs_time::TimeFrame(start_time, end_time);
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 const interval_t &linear_interval,
                 const interval_t &angular_interval,
                 typename cslibs_math_2d::PolarPointcloud2<T>::Ptr &dst)
{
    const float range_min = std::max(linear_interval[0],  src->range_min);
    const float range_max = std::min(linear_interval[1],  src->range_max);
    const float angle_min = std::max(angular_interval[0], src->angle_min);
    const float angle_max = std::min(angular_interval[1], src->angle_max);

    auto in_linear_interval = [range_min, range_max](const float range)
    {
        return range >= range_min && range <= range_max;
    };
    auto in_angular_interval = [angle_min, angle_max](const float angle)
    {
        return angle >= angle_min && angle <= angle_max;
    };


    dst.reset(new cslibs_math_2d::PolarPointcloud2<T>);
    const float angle_incr = src->angle_increment;
    float angle = angle_min;
    for(const float range : src->ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle)) {
            const cslibs_math_2d::PolarPoint2<T> p(static_cast<T>(angle),
                                                   static_cast<T>(range));
            dst->insert(p);
        } else {
            dst->insertInvalid();
        }
        angle += angle_incr;
    };
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 typename cslibs_math_2d::PolarPointcloud2<T>::Ptr &dst)
{
    from(src,
         {{src->range_min, src->range_max}},
         {{src->angle_min, src->angle_max}},
         dst);
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 const interval_t &linear_interval,
                 const interval_t &angular_interval,
                 typename cslibs_math_2d::Pointcloud2<T>::Ptr &dst)
{
    const float range_min = std::max(linear_interval[0],  src->range_min);
    const float range_max = std::min(linear_interval[1],  src->range_max);
    const float angle_min = std::max(angular_interval[0], src->angle_min);
    const float angle_max = std::min(angular_interval[1], src->angle_max);

    auto in_linear_interval = [range_min, range_max](const float range)
    {
        return range >= range_min && range <= range_max;
    };
    auto in_angular_interval = [angle_min, angle_max](const float angle)
    {
        return angle >= angle_min && angle <= angle_max;
    };


    dst.reset(new cslibs_math_2d::Pointcloud2<T>);
    const float angle_incr = src->angle_increment;
    float angle = angle_min;
    for(const float range : src->ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle)) {
            const cslibs_math_2d::Point2<T> p(static_cast<T>(std::cos(angle) * range),
                                               static_cast<T>(std::sin(angle) * range));
            dst->insert(p);
        } else {
            dst->insertInvalid();
        }
        angle += angle_incr;
    };
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 typename cslibs_math_2d::Pointcloud2<T>::Ptr &dst)

{
    from(src,
         {{src->range_min, src->range_max}},
         {{src->angle_min, src->angle_max}},
         dst);
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 const interval_t    &linear_interval,
                 const interval_t    &angular_interval,
                 const std::string   &fixed_frame,
                 const ros::Duration &tf_timeout,
                 cslibs_math_ros::tf::TFListener &tfl,
                 typename cslibs_math_2d::PolarPointcloud2<T>::Ptr &dst)
{
    const float range_min  = std::max(src->range_min, linear_interval[0]);
    const float range_max  = std::min(src->range_max, linear_interval[1]);
    const float angle_min  = std::max(src->angle_min, angular_interval[0]);
    const float angle_max  = std::min(src->angle_max, angular_interval[1]);
    const float angle_incr = src->angle_increment;

    if(src->ranges.size() == 0ul)
        return;

    dst.reset(new cslibs_math_2d::PolarPointcloud2<T>);

    auto in_linear_interval = [range_min, range_max](const float range)
    {
        return range >= range_min && range <= range_max;
    };
    auto in_angular_interval = [angle_min, angle_max](const float angle)
    {
        return angle >= angle_min && angle <= angle_max;
    };


    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(static_cast<double>(src->time_increment));
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(static_cast<double>(src->scan_time / static_cast<float>(src->ranges.size())));
    }
    const ros::Time end_stamp = start_stamp + delta_stamp * src->ranges.size();

    cslibs_math_2d::Transform2<T> f_T_end;
    cslibs_math_2d::Transform2<T> f_T_start;
    if(!tfl.lookupTransform(fixed_frame, src->header.frame_id, end_stamp, f_T_end, tf_timeout)) {
        return;
    }
    if(!tfl.lookupTransform(fixed_frame, src->header.frame_id, start_stamp, f_T_start, tf_timeout)) {
        return;
    }

    cslibs_math_2d::Transform2<T> end_T_start = f_T_end.inverse() * f_T_start;
    auto  angle = angle_min;
    double dt = 0.0;
    for(const auto range : src->ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle)) {
            cslibs_math_2d::Transform2<T>  Tf = cslibs_math_2d::Transform2<T>::identity().interpolate(end_T_start, dt);
            cslibs_math_2d::PolarPoint2<T> pt = Tf * cslibs_math_2d::Point2<T>(std::cos(angle) * range,
                                                                                 std::sin(angle) * range);
            dst->insert(pt);
        } else {
            dst->insertInvalid();
        }
        angle += angle_incr;
        dt    += delta_stamp.toSec();
    }
}

template <typename T>
inline void from(const ::sensor_msgs::LaserScan::ConstPtr &src,
                 const std::string &fixed_frame,
                 const ros::Duration &tf_timeout,
                 cslibs_math_ros::tf::TFListener &tfl,
                 typename cslibs_math_2d::PolarPointcloud2<T>::Ptr &dst)
{
    from(src,
         {{src->range_min, src->range_max}},
         {{src->angle_min, src->angle_max}},
         fixed_frame,
         tf_timeout,
         tfl,
         dst);
}
}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_2D_HPP
