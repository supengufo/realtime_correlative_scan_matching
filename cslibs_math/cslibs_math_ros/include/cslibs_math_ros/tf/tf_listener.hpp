#pragma once

#include <cslibs_math_ros/tf/tf_provider.hpp>

#include <cslibs_math_ros/tf/conversion_2d.hpp>
#include <cslibs_math_ros/tf/conversion_3d.hpp>
#include <cslibs_time/conversion/ros.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mutex>


namespace cslibs_math_ros {
namespace tf {
class TFListener : public TFProvider
{
public:
    using Ptr           = std::shared_ptr<TFListener>;
    using mutex_t       = std::mutex;
    using lock_t        = std::unique_lock<mutex_t>;

    // inherit fallbacks
    using TFProvider::lookupTransform;

    // 2d
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_2d_t<float>& transform,
                         const ros::Duration& timeout) override
    {
        ::tf::Transform tf_transform;
        if (!lookupTransform(target_frame, source_frame, time, tf_transform, timeout))
            return false;

        transform.data() = conversion_2d::from<float>(tf_transform);
        transform.stamp() = cslibs_time::from(time);
        return true;
    }
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_2d_t<double>& transform,
                         const ros::Duration& timeout) override
    {
        ::tf::Transform tf_transform;
        if (!lookupTransform(target_frame, source_frame, time, tf_transform, timeout))
            return false;

        transform.data() = conversion_2d::from<double>(tf_transform);
        transform.stamp() = cslibs_time::from(time);
        return true;
    }

    // 3d
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_3d_t<float>& transform,
                         const ros::Duration& timeout) override
    {
        ::tf::Transform tf_transform;
        if (!lookupTransform(target_frame, source_frame, time, tf_transform, timeout))
            return false;

        transform.data() = conversion_3d::from<float>(tf_transform);
        transform.stamp() = cslibs_time::from(time);
        return true;
    }
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_3d_t<double>& transform,
                         const ros::Duration& timeout) override
    {
        ::tf::Transform tf_transform;
        if (!lookupTransform(target_frame, source_frame, time, tf_transform, timeout))
            return false;

        transform.data() = conversion_3d::from<double>(tf_transform);
        transform.stamp() = cslibs_time::from(time);
        return true;
    }

    // tf
    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         ::tf::StampedTransform& transform,
                         const ros::Duration& timeout = {})
    {
        assert(!target_frame.empty());
        assert(!source_frame.empty());

        auto check_tf = [&]()
        {
            return timeout.isZero() ?
                   tf_.canTransform(target_frame, source_frame, time) :
                   tf_.waitForTransform(target_frame, source_frame, time, timeout);
        };

        lock_t l(mutex_);
        if (!check_tf())
            return false;

        tf_.lookupTransform(target_frame, source_frame, time, transform);
        return true;
    }


    bool lookupTransform(const std::string& target_frame,
                                const std::string& source_frame,
                                const ros::Time& time,
                                ::tf::Transform& transform,
                                const ros::Duration& timeout = {})
    {
        ::tf::StampedTransform stamped;
        if (!lookupTransform(target_frame, source_frame, time, stamped, timeout))
            return false;

        transform = stamped;
        return true;
    }

    bool canTransform(const std::string& target_frame,
                      const std::string& source_frame,
                      const ros::Time& time) override
    {
        assert(!target_frame.empty());
        assert(!source_frame.empty());

        lock_t l(mutex_);
        return tf_.canTransform(target_frame, source_frame, time);
    }

    bool waitForTransform(const std::string& target_frame,
                          const std::string& source_frame,
                          const ros::Time& time,
                          const ros::Duration& timeout) override
    {
        assert(!target_frame.empty());
        assert(!source_frame.empty());

        lock_t l(mutex_);
        return tf_.waitForTransform(target_frame, source_frame, time, timeout);
    }

protected:
    std::mutex              mutex_;
    ::tf::TransformListener tf_;
};
}
}
