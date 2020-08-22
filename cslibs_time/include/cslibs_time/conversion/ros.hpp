#ifndef CSLIBS_TIME_ROS_HPP
#define CSLIBS_TIME_ROS_HPP

#include <ros/rate.h>
#include <ros/time.h>

#include <cslibs_time/time.hpp>
#include <cslibs_time/rate.hpp>
#include <cslibs_time/duration.hpp>
#include <cslibs_time/time_frame.hpp>


namespace cslibs_time {
using ros_time_frame_t = std::pair<ros::Time, ros::Time>;

/// FROM ROS
inline Time from(const ros::Time &t)
{
    return Time(t.toNSec());
}

inline Duration from(const ros::Duration &d)
{
    return Duration(d.toNSec());
}

inline TimeFrame from(const ros::Time &t_start,
                      const ros::Time &t_end)
{
    return TimeFrame(from(t_start),
                     from(t_end));
}

inline TimeFrame from(const ros_time_frame_t &f)
{
    return TimeFrame(from(f.first),
                     from(f.second));
}

inline Rate from(const ros::Rate &r)
{
    return Rate(from(r.expectedCycleTime()));
}

inline void from(const ros::Time &t_src,
                 Time &t_dst)
{
    t_dst = Time(t_src.toNSec());
}

inline void from(const ros::Duration &d_src,
                 Duration &d_dst)
{
    d_dst = Duration(d_src.toNSec());
}
/*
inline void from(const ros::Time &t_start,
                 const ros::Time &t_end,
                 TimeFrame &f)
{
    f = TimeFrame(from(t_start),
                  from(t_end));
}
*/
inline void from(const ros::Rate &r_src,
                 Rate r_dst)
{
    r_dst = Rate(from(r_src.expectedCycleTime()));
}

/// TO ROS
inline ros::Time from(const Time &t)
{
    const int64_t  ns = t.nanoseconds();
    const uint32_t s  = ns / static_cast<uint32_t>(1e9);
    const uint32_t n = ns - s * 1e9;
    return ros::Time(s,n);
}

inline ros::Duration from(const Duration &d)
{
    const int64_t  ns = d.nanoseconds();
    const uint32_t s  = ns / static_cast<uint32_t>(1e9);
    const uint32_t n = ns - s * 1e9;
    return ros::Duration(s,n);
}

inline ros_time_frame_t from(const TimeFrame &t)
{
    return {from(t.start),
            from(t.end)};
}

inline ros::Rate from(const Rate &r)
{
    return ros::Rate(r.frequency());
}

inline void from(const Time &t_src,
                 ros::Time  &t_dst)
{
    t_dst.fromNSec(t_src.nanoseconds());
}

inline void from(const Duration &d_src,
                 ros::Duration &d_dst)
{
    d_dst.fromNSec(d_src.nanoseconds());
}

inline void from(const TimeFrame &f,
                 ros::Time &t_start,
                 ros::Time &t_end)
{
    t_start.fromNSec(f.start.nanoseconds());
    t_end.fromNSec(f.end.nanoseconds());
}

inline void from(const Rate &r_src,
                 ros::Rate r_dst)
{
    r_dst = ros::Rate(from(r_src.expectedCycleTime()));
}
}
#endif // CSLIBS_TIME_ROS_HPP
