#ifndef CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
#define CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cslibs_time/time_frame.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>

namespace cslibs_math_ros {
namespace sensor_msgs {
namespace conversion_3d {
inline cslibs_time::TimeFrame from(const ::sensor_msgs::PointCloud2ConstPtr &src)
{
    const ros::Time start_stamp = src->header.stamp;
    return cslibs_time::TimeFrame(start_stamp.toNSec(),
                                  start_stamp.toNSec());
}

template <typename T>
inline void from(const ::sensor_msgs::PointCloud2ConstPtr &src,
                 typename cslibs_math_3d::Pointcloud3<T>::Ptr &dst)
{
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_x(*src, "x");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_y(*src, "y");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_z(*src, "z");

    dst.reset(new cslibs_math_3d::Pointcloud3<T>);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
            cslibs_math_3d::Point3<T> p(*iter_x, *iter_y, *iter_z);
            if (p.isNormal())
                dst->insert(p);
        }
    }
}

template <typename T>
inline void from(const typename cslibs_math_3d::Pointcloud3<T>::Ptr &src,
                 ::sensor_msgs::PointCloud2 &dst)
{
    // metadata
    dst.width        = src->size();
    dst.height       = 1;
    dst.is_dense     = false;
    dst.is_bigendian = false;


    ::sensor_msgs::PointCloud2Modifier modifier(dst);
    modifier.setPointCloud2FieldsByString(1,"xyz");
    modifier.resize(src->size());

    ::sensor_msgs::PointCloud2Iterator<float> iter_x(dst, "x");
    ::sensor_msgs::PointCloud2Iterator<float> iter_y(dst, "y");
    ::sensor_msgs::PointCloud2Iterator<float> iter_z(dst, "z");

    for (const auto &p : *src) {
        *iter_x = static_cast<float>(p(0));
        *iter_y = static_cast<float>(p(1));
        *iter_z = static_cast<float>(p(2));
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
}

template <typename T>
inline void from(const ::sensor_msgs::PointCloud2ConstPtr &src,
                 typename cslibs_math_3d::PointcloudRGB3<T>::Ptr &dst)
{
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_x(*src, "x");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_y(*src, "y");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_z(*src, "z");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_r(*src, "r");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_g(*src, "g");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_b(*src, "b");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_a(*src, "a");

    dst.reset(new cslibs_math_3d::PointcloudRGB3<T>);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iter_a) {
        if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
            cslibs_math_3d::Point3<T> p(*iter_x, *iter_y, *iter_z);
            cslibs_math::color::Color<T> c(static_cast<float>(*iter_r)/256.0f,
                                           static_cast<float>(*iter_g)/256.0f,
                                           static_cast<float>(*iter_b)/256.0f);
            cslibs_math_3d::PointRGB3<T> point(p, static_cast<float>(*iter_a)/256.0, c);
            if (p.isNormal())
                dst->insert(point);
        }
    }
}

template <typename T>
inline void from(const typename cslibs_math_3d::PointcloudRGB3<T>::Ptr &src,
                 ::sensor_msgs::PointCloud2 &dst)
{
    // metadata
    dst.width        = src->size();
    dst.height       = 1;
    dst.is_dense     = false;
    dst.is_bigendian = false;


    ::sensor_msgs::PointCloud2Modifier modifier(dst);
    modifier.setPointCloud2FieldsByString(2,"xyz","rgba");
    modifier.resize(src->size());

    ::sensor_msgs::PointCloud2Iterator<float> iter_x(dst, "x");
    ::sensor_msgs::PointCloud2Iterator<float> iter_y(dst, "y");
    ::sensor_msgs::PointCloud2Iterator<float> iter_z(dst, "z");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_r(dst, "r");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_g(dst, "g");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_b(dst, "b");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_a(dst, "a");

    for (const auto &p : *src) {
        cslibs_math_3d::Point3<T> pos = p.getPoint();
        cslibs_math::color::Color<T> c = p.getColor();
        float a = p.getAlpha();
        *iter_x = static_cast<float>(pos(0));
        *iter_y = static_cast<float>(pos(1));
        *iter_z = static_cast<float>(pos(2));
        *iter_r = static_cast<u_int8_t>(c.r*255.0f);
        *iter_g = static_cast<u_int8_t>(c.g*255.0f);
        *iter_b = static_cast<u_int8_t>(c.b*255.0f);
        *iter_a = static_cast<u_int8_t>(a*255.0f);
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
        ++iter_a;
    }
}

}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
