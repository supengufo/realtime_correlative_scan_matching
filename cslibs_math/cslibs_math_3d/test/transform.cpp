#include <cslibs_math/random/random.hpp>
#include <cslibs_math/common/angle.hpp>
#include <cslibs_math/common/equal.hpp>

#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_math_3d/linear/point.hpp>
//#include <cslibs_math_3d/conversion/tf.hpp>

#include <gtest/gtest.h>
#include <tf/tf.h>

using rng_t = cslibs_math::random::Uniform<double,1>;

TEST(Test_cslibs_math_3d, testTransformInitEye)
{
    cslibs_math_3d::Transform3d t_0;
    EXPECT_EQ(t_0.roll(),   0.0);
    EXPECT_EQ(t_0.pitch(),  0.0);
    EXPECT_EQ(t_0.yaw(),    0.0);
    EXPECT_EQ(t_0.tx(),     0.0);
    EXPECT_EQ(t_0.ty(),     0.0);
    EXPECT_EQ(t_0.tz(),     0.0);

    cslibs_math_3d::Transform3d t_1 = cslibs_math_3d::Transform3d(0.0, 0.0, 0.0);
    EXPECT_EQ(t_1.roll(),   0.0);
    EXPECT_EQ(t_1.pitch(),  0.0);
    EXPECT_EQ(t_1.yaw(),    0.0);
    EXPECT_EQ(t_1.tx(),     0.0);
    EXPECT_EQ(t_1.ty(),     0.0);
    EXPECT_EQ(t_1.tz(),     0.0);

    cslibs_math_3d::Transform3d t_2 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(0.0, 0.0, 0.0));
    EXPECT_EQ(t_2.roll(),   0.0);
    EXPECT_EQ(t_2.pitch(),  0.0);
    EXPECT_EQ(t_2.yaw(),    0.0);
    EXPECT_EQ(t_2.tx(),     0.0);
    EXPECT_EQ(t_2.ty(),     0.0);
    EXPECT_EQ(t_2.tz(),     0.0);

    cslibs_math_3d::Transform3d t_3 = cslibs_math_3d::Transform3d(0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(t_3.roll(),   0.0);
    EXPECT_EQ(t_3.pitch(),  0.0);
    EXPECT_EQ(t_3.yaw(),    0.0);
    EXPECT_EQ(t_3.tx(),     0.0);
    EXPECT_EQ(t_3.ty(),     0.0);
    EXPECT_EQ(t_3.tz(),     0.0);

    cslibs_math_3d::Transform3d t_4 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(0.0, 0.0, 0.0), 0.0);
    EXPECT_EQ(t_4.roll(),   0.0);
    EXPECT_EQ(t_4.pitch(),  0.0);
    EXPECT_EQ(t_4.yaw(),    0.0);
    EXPECT_EQ(t_4.tx(),     0.0);
    EXPECT_EQ(t_4.ty(),     0.0);
    EXPECT_EQ(t_4.tz(),     0.0);
}

TEST(Test_cslibs_math_3d, testTransformInitTranslation)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double z = rng.get();

    cslibs_math_3d::Transform3d t_0 = cslibs_math_3d::Transform3d(x, y, z);
    EXPECT_EQ(t_0.roll(),   0.0);
    EXPECT_EQ(t_0.pitch(),  0.0);
    EXPECT_EQ(t_0.yaw(),    0.0);
    EXPECT_EQ(t_0.tx(),     x);
    EXPECT_EQ(t_0.ty(),     y);
    EXPECT_EQ(t_0.tz(),     z);

    cslibs_math_3d::Transform3d t_1 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(x, y, z));
    EXPECT_EQ(t_1.roll(),   0.0);
    EXPECT_EQ(t_1.pitch(),  0.0);
    EXPECT_EQ(t_1.yaw(),    0.0);
    EXPECT_EQ(t_1.tx(),     x);
    EXPECT_EQ(t_1.ty(),     y);
    EXPECT_EQ(t_1.tz(),     z);

    cslibs_math_3d::Transform3d t_2 = cslibs_math_3d::Transform3d(x, y, z, 0.0);
    EXPECT_EQ(t_2.roll(),   0.0);
    EXPECT_EQ(t_2.pitch(),  0.0);
    EXPECT_EQ(t_2.yaw(),    0.0);
    EXPECT_EQ(t_2.tx(),     x);
    EXPECT_EQ(t_2.ty(),     y);
    EXPECT_EQ(t_2.tz(),     z);

    cslibs_math_3d::Transform3d t_3 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(x, y, z), 0.0);
    EXPECT_EQ(t_3.roll(),   0.0);
    EXPECT_EQ(t_3.pitch(),  0.0);
    EXPECT_EQ(t_3.yaw(),    0.0);
    EXPECT_EQ(t_3.tx(),     x);
    EXPECT_EQ(t_3.ty(),     y);
    EXPECT_EQ(t_3.tz(),     z);
}

TEST(Test_cslibs_math_3d, testTransformInitRotation)
{
    rng_t rng(-10.0, 10.0);
    const double roll  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch = cslibs_math::common::angle::normalize(rng.get());
    const double yaw   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_0 = cslibs_math_3d::Transform3d(0.0, 0.0, 0.0, yaw);
    EXPECT_EQ(t_0.roll(),  0.0);
    EXPECT_EQ(t_0.pitch(), 0.0);
    EXPECT_NEAR(t_0.yaw(), yaw, 1e-6);
    EXPECT_EQ(t_0.tx(),    0.0);
    EXPECT_EQ(t_0.ty(),    0.0);
    EXPECT_EQ(t_0.tz(),    0.0);

    cslibs_math_3d::Transform3d t_1 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(), yaw);
    EXPECT_EQ(t_1.tx(),  0.0);
    EXPECT_EQ(t_1.ty(),  0.0);
    EXPECT_EQ(t_1.tz(),  0.0);
    EXPECT_EQ(t_1.roll(),  0.0);
    EXPECT_EQ(t_1.pitch(), 0.0);
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_1.yaw()), M_PI, 1e-6));

    cslibs_math_3d::Transform3d t_2 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(),
                                                                  cslibs_math_3d::Quaterniond(roll, pitch, yaw));
    EXPECT_EQ(t_2.tx(),  0.0);
    EXPECT_EQ(t_2.ty(),  0.0);
    EXPECT_EQ(t_2.tz(),  0.0);
    EXPECT_TRUE(cslibs_math::common::eq(roll, t_2.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll) + std::abs(t_2.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch, t_2.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch + t_2.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_2.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_2.yaw()), M_PI, 1e-6));
}

TEST(Test_cslibs_math_3d, testTransformConstructors)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double z = rng.get();
    const double roll  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch = cslibs_math::common::angle::normalize(rng.get());
    const double yaw   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_0 = cslibs_math_3d::Transform3d(x, y, z, yaw);
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_0.yaw()), M_PI, 1e-6));
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.tz(),  z);


    cslibs_math_3d::Transform3d t_1 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(x, y, z), yaw);
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_1.yaw()), M_PI, 1e-6));
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.tz(),  z);

    cslibs_math_3d::Transform3d t_2 = cslibs_math_3d::Transform3d(cslibs_math_3d::Point3d(x, y, z),
                                                                  cslibs_math_3d::Quaterniond(roll, pitch, yaw));
    EXPECT_EQ(t_2.tx(),  x);
    EXPECT_EQ(t_2.ty(),  y);
    EXPECT_EQ(t_2.tz(),  z);
    EXPECT_TRUE(cslibs_math::common::eq(roll, t_2.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll) + std::abs(t_2.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch, t_2.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch + t_2.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_2.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_2.yaw()), M_PI, 1e-6));
}

TEST(Test_cslibs_math_3d, testTransformSetFrom)
{
    rng_t rng(-10.0, 10.0);
    const double x = rng.get();
    const double y = rng.get();
    const double z = rng.get();
    const double roll  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch = cslibs_math::common::angle::normalize(rng.get());
    const double yaw   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d::eigen_vector_6d_t e;
    e(0) = x;
    e(1) = y;
    e(2) = z;
    e(3) = roll;
    e(4) = pitch;
    e(5) = yaw;
    cslibs_math_3d::Transform3d t_0;
    t_0.setFrom(e);
    EXPECT_EQ(t_0.tx(),  x);
    EXPECT_EQ(t_0.ty(),  y);
    EXPECT_EQ(t_0.tz(),  z);
    EXPECT_TRUE(cslibs_math::common::eq(roll, t_0.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll) + std::abs(t_0.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch, t_0.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch + t_0.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_0.yaw()), M_PI, 1e-6));

    cslibs_math_3d::Transform3d t_1;
    t_1.setFrom(x,y,z,roll,pitch,yaw);
    EXPECT_EQ(t_1.tx(),  x);
    EXPECT_EQ(t_1.ty(),  y);
    EXPECT_EQ(t_1.tz(),  z);
    EXPECT_TRUE(cslibs_math::common::eq(roll, t_1.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll) + std::abs(t_1.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch, t_1.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch + t_1.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw) + std::abs(t_1.yaw()), M_PI, 1e-6));
}


TEST(Test_cslibs_math_3d, testTransformSetRPY)
{
    rng_t rng(-10.0, 10.0);
    const double roll  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch = cslibs_math::common::angle::normalize(rng.get());
    const double yaw0  = cslibs_math::common::angle::normalize(rng.get());
    const double yaw1  = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_0;
    EXPECT_EQ(t_0.roll(),  0.0);
    EXPECT_EQ(t_0.pitch(), 0.0);
    EXPECT_EQ(t_0.yaw(),   0.0);

    t_0.setYaw(yaw0);
    EXPECT_TRUE(cslibs_math::common::eq(yaw0, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw0) + std::abs(t_0.yaw()), M_PI, 1e-6));
    t_0.setRPY(roll, pitch, yaw1);
    EXPECT_TRUE(cslibs_math::common::eq(roll, t_0.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll) + std::abs(t_0.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch, t_0.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch + t_0.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw1, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw1) + std::abs(t_0.yaw()), M_PI, 1e-6));
}

TEST(Test_cslibs_math_3d, testTransformTranslation)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double z_0 = rng.get();
    const double p_x = rng.get();
    const double p_y = rng.get();
    const double p_z = rng.get();

    cslibs_math_3d::Transform3d t_0(x_0,y_0,z_0);
    EXPECT_EQ(t_0.tx(), x_0);
    EXPECT_EQ(t_0.ty(), y_0);
    EXPECT_EQ(t_0.tz(), z_0);
    EXPECT_EQ(t_0.roll(),  0.0);
    EXPECT_EQ(t_0.pitch(), 0.0);
    EXPECT_EQ(t_0.yaw(),   0.0);

    tf::Transform t_0_tf(tf::createIdentityQuaternion(),
                         tf::Vector3(x_0,y_0,z_0));

    cslibs_math_3d::Point3d p_0(p_x, p_y, p_z);
    tf::Point               p_tf(p_x, p_y, p_z);

    p_0  = t_0 * p_0;
    p_tf = t_0_tf * p_tf;
    EXPECT_EQ(p_0(0), p_x + x_0);
    EXPECT_EQ(p_0(1), p_y + y_0);
    EXPECT_EQ(p_0(2), p_z + z_0);
    EXPECT_EQ(p_0(0), p_tf.x());
    EXPECT_EQ(p_0(1), p_tf.y());
    EXPECT_EQ(p_0(2), p_tf.z());

    const double x_1 = rng.get();
    const double y_1 = rng.get();
    const double z_1 = rng.get();
    cslibs_math_3d::Point3d p_1(x_1, y_1, z_1);
    cslibs_math_3d::Transform3d t_1(x_1,y_1,z_1);
    tf::Transform t_1_tf(tf::createIdentityQuaternion(),
                         tf::Vector3(x_1,y_1,z_1));

    p_1 = t_0 * p_1;
    EXPECT_EQ(p_1(0), x_1 + x_0);
    EXPECT_EQ(p_1(1), y_1 + y_0);
    EXPECT_EQ(p_1(2), z_1 + z_0);
    EXPECT_EQ(t_1.roll(),  0.0);
    EXPECT_EQ(t_1.pitch(), 0.0);
    EXPECT_EQ(t_1.yaw(),   0.0);
    EXPECT_EQ(t_0.roll(),  0.0);
    EXPECT_EQ(t_0.pitch(), 0.0);
    EXPECT_EQ(t_0.yaw(),   0.0);
    t_1    = t_0 * t_1;
    t_1_tf = t_0_tf * t_1_tf;
    EXPECT_EQ(t_1.tx(), x_1 + x_0);
    EXPECT_EQ(t_1.ty(), y_1 + y_0);
    EXPECT_EQ(t_1.tz(), z_1 + z_0);
    EXPECT_EQ(t_1.tx(), t_1_tf.getOrigin().x());
    EXPECT_EQ(t_1.ty(), t_1_tf.getOrigin().y());
    EXPECT_EQ(t_1.tz(), t_1_tf.getOrigin().z());
}

TEST(Test_cslibs_math_3d, testTransformRotation)
{
    rng_t rng(-10.0, 10.0);
    const double roll_0  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_0 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_0   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_0(cslibs_math_3d::Vector3d(0.0),
                                    cslibs_math_3d::Quaterniond(roll_0, pitch_0, yaw_0));
    EXPECT_TRUE(cslibs_math::common::eq(roll_0, t_0.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll_0) + std::abs(t_0.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch_0, t_0.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch_0 + t_0.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw_0, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw_0) + std::abs(t_0.yaw()), M_PI, 1e-6));
    EXPECT_EQ(t_0.tx(), 0.0);
    EXPECT_EQ(t_0.ty(), 0.0);
    EXPECT_EQ(t_0.tz(), 0.0);

    tf::Transform t_0_tf = tf::Transform(tf::createQuaternionFromRPY(roll_0, pitch_0, yaw_0),
                                         tf::Vector3(0.0,0.0,0.0));
    cslibs_math_3d::Point3d p = cslibs_math_3d::Point3d::random();
    tf::Point p_tf(p(0), p(1), p(2));

    EXPECT_EQ(p(0), p_tf.x());
    EXPECT_EQ(p(1), p_tf.y());
    EXPECT_EQ(p(2), p_tf.z());

    p    = t_0 * p;
    p_tf = t_0_tf * p_tf;

    EXPECT_NEAR(p(0), p_tf.x(), 1e-5);
    EXPECT_NEAR(p(1), p_tf.y(), 1e-5);
    EXPECT_NEAR(p(2), p_tf.z(), 1e-5);

    const double roll_1  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_1 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_1   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_1(cslibs_math_3d::Vector3d(0.0),
                                    cslibs_math_3d::Quaterniond(roll_1, pitch_1, yaw_1));
    tf::Transform t_1_tf(tf::createQuaternionFromRPY(roll_1, pitch_1, yaw_1),
                         tf::Vector3(0.0,0.0,0.0));

    EXPECT_EQ(t_1.tx(), 0.0);
    EXPECT_EQ(t_1.ty(), 0.0);
    EXPECT_EQ(t_1.tz(), 0.0);

    t_1 = t_0 * t_1;
    t_1_tf = t_0_tf * t_1_tf;

    EXPECT_EQ(t_0.tx(), 0.0);
    EXPECT_EQ(t_0.ty(), 0.0);
    EXPECT_EQ(t_0.tz(), 0.0);
    EXPECT_EQ(t_1.tx(), 0.0);
    EXPECT_EQ(t_1.ty(), 0.0);
    EXPECT_EQ(t_1.tz(), 0.0);


    double t_1_tf_roll, t_1_tf_pitch, t_1_tf_yaw;
    tf::Matrix3x3(t_1_tf.getRotation()).getRPY(t_1_tf_roll,
                                               t_1_tf_pitch,
                                               t_1_tf_yaw);

    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_roll, t_1.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_roll) + std::abs(t_1.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_pitch, t_1.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_pitch + t_1.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_yaw) + std::abs(t_0.yaw()), M_PI, 1e-6));

}

TEST(Test_cslibs_math_3d, testTransformFull)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double z_0 = rng.get();
    const double roll_0  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_0 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_0   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_0(cslibs_math_3d::Vector3d(x_0, y_0, z_0),
                                    cslibs_math_3d::Quaterniond(roll_0, pitch_0, yaw_0));
    tf::Transform t_0_tf(tf::createQuaternionFromRPY(roll_0, pitch_0, yaw_0),
                         tf::Vector3(x_0, y_0, z_0));

    EXPECT_EQ(t_0.tx(), x_0);
    EXPECT_EQ(t_0.ty(), y_0);
    EXPECT_EQ(t_0.tz(), z_0);
    EXPECT_TRUE(cslibs_math::common::eq(roll_0, t_0.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll_0) + std::abs(t_0.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch_0, t_0.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch_0 + t_0.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw_0, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw_0) + std::abs(t_0.yaw()), M_PI, 1e-6));

    cslibs_math_3d::Point3d p = cslibs_math_3d::Point3d::random();
    tf::Point p_tf(p(0), p(1), p(2));
    EXPECT_EQ(p(0), p_tf.x());
    EXPECT_EQ(p(1), p_tf.y());
    EXPECT_EQ(p(2), p_tf.z());

    p    = t_0 * p;
    p_tf = t_0_tf * p_tf;

    EXPECT_NEAR(p(0), p_tf.x(), 1e-5);
    EXPECT_NEAR(p(1), p_tf.y(), 1e-5);
    EXPECT_NEAR(p(2), p_tf.z(), 1e-5);

    const double x_1 = rng.get();
    const double y_1 = rng.get();
    const double z_1 = rng.get();
    const double roll_1  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_1 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_1   = cslibs_math::common::angle::normalize(rng.get());

    cslibs_math_3d::Transform3d t_1(cslibs_math_3d::Vector3d(x_1, y_1, z_1),
                                    cslibs_math_3d::Quaterniond(roll_1, pitch_1, yaw_1));
    tf::Transform t_1_tf(tf::createQuaternionFromRPY(roll_1, pitch_1, yaw_1),
                         tf::Vector3(x_1, y_1, z_1));

    EXPECT_EQ(t_1.tx(), x_1);
    EXPECT_EQ(t_1.ty(), y_1);
    EXPECT_EQ(t_1.tz(), z_1);
    EXPECT_TRUE(cslibs_math::common::eq(roll_1, t_1.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(roll_1) + std::abs(t_1.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(pitch_1, t_1.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(pitch_1 + t_1.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(yaw_1, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(yaw_1) + std::abs(t_1.yaw()), M_PI, 1e-6));


    t_1 = t_0 * t_1;
    t_1_tf = t_0_tf * t_1_tf;
    double t_1_tf_roll, t_1_tf_pitch, t_1_tf_yaw;
    tf::Matrix3x3(t_1_tf.getRotation()).getRPY(t_1_tf_roll,
                                               t_1_tf_pitch,
                                               t_1_tf_yaw);

    EXPECT_NEAR(t_1.tx(), t_1_tf.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_1.ty(), t_1_tf.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_1.tz(), t_1_tf.getOrigin().z(), 1e-5);
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_roll, t_1.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_roll) + std::abs(t_1.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_pitch, t_1.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_pitch + t_1.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_yaw) + std::abs(t_1.yaw()), M_PI, 1e-6));

    cslibs_math_3d::Transform3d t_2 = t_0;
    t_2 *= t_1; /// t_2 = t_0 * t_1;
    t_1  = t_0 * t_1;
    EXPECT_NEAR(t_2.tx(),    t_1.tx(), 1e-5);
    EXPECT_NEAR(t_2.ty(),    t_1.ty(), 1e-5);
    EXPECT_NEAR(t_2.tz(),    t_1.tz(), 1e-5);
    EXPECT_NEAR(t_2.roll(),  t_1.roll(), 1e-5);
    EXPECT_NEAR(t_2.pitch(), t_1.pitch(), 1e-5);
    EXPECT_NEAR(t_2.yaw(),   t_1.yaw(), 1e-5);

}

TEST(Test_cslibs_math_3d, testTransformInterpolation)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double z_0 = rng.get();
    const double x_1 = rng.get();
    const double y_1 = rng.get();
    const double z_1 = rng.get();
    const double roll_0  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_0 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_0   = cslibs_math::common::angle::normalize(rng.get());
    const double roll_1  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_1 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_1   = cslibs_math::common::angle::normalize(rng.get());


    tf::Transform t_0_tf(tf::createQuaternionFromRPY(roll_0, pitch_0, yaw_0),
                       tf::Vector3(x_0, y_0, z_0));
    tf::Transform t_1_tf(tf::createQuaternionFromRPY(roll_1, pitch_1, yaw_1),
                       tf::Vector3(x_1, y_1, z_1));
    cslibs_math_3d::Transform3d t_0(cslibs_math_3d::Vector3d(x_0, y_0, z_0),
                                    cslibs_math_3d::Quaterniond(roll_0, pitch_0, yaw_0));
    cslibs_math_3d::Transform3d t_1(cslibs_math_3d::Vector3d(x_1, y_1, z_1),
                                    cslibs_math_3d::Quaterniond(roll_1, pitch_1, yaw_1));


    double t_0_tf_roll, t_0_tf_pitch, t_0_tf_yaw;
    tf::Matrix3x3(t_0_tf.getRotation()).getRPY(t_0_tf_roll,
                                               t_0_tf_pitch,
                                               t_0_tf_yaw);
    double t_1_tf_roll, t_1_tf_pitch, t_1_tf_yaw;
    tf::Matrix3x3(t_1_tf.getRotation()).getRPY(t_1_tf_roll,
                                               t_1_tf_pitch,
                                               t_1_tf_yaw);

    EXPECT_NEAR(t_0.tx(), t_0_tf.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_0.ty(), t_0_tf.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_0.tz(), t_0_tf.getOrigin().z(), 1e-5);
    EXPECT_TRUE(cslibs_math::common::eq(t_0_tf_roll, t_0.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_0_tf_roll) + std::abs(t_0.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_0_tf_pitch, t_0.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_0_tf_pitch + t_0.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_0_tf_yaw, t_0.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_0_tf_yaw) + std::abs(t_0.yaw()), M_PI, 1e-6));

    EXPECT_NEAR(t_1.tx(), t_1_tf.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_1.ty(), t_1_tf.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_1.tz(), t_1_tf.getOrigin().z(), 1e-5);
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_roll, t_1.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_roll) + std::abs(t_1.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_pitch, t_1.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_pitch + t_1.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_1_tf_yaw, t_1.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_1_tf_yaw) + std::abs(t_1.yaw()), M_PI, 1e-6));

    cslibs_math_3d::Transform3d t_r = t_0.interpolate(t_1, 0.66);
    tf::Transform t_r_tf;
    t_r_tf.getOrigin().setInterpolate3(t_0_tf.getOrigin(), t_1_tf.getOrigin(), 0.66);
    t_r_tf.setRotation(tf::slerp(t_0_tf.getRotation(), t_1_tf.getRotation(), 0.66));

    double t_r_tf_roll, t_r_tf_pitch, t_r_tf_yaw;
    tf::Matrix3x3(t_r_tf.getRotation()).getRPY(t_r_tf_roll,
                                               t_r_tf_pitch,
                                               t_r_tf_yaw);
    EXPECT_NEAR(t_r.tx(), t_r_tf.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_r.ty(), t_r_tf.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_r.tz(), t_r_tf.getOrigin().z(), 1e-5);
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_roll, t_r.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_roll) + std::abs(t_r.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_pitch, t_r.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_pitch + t_r.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_yaw, t_r.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_yaw) + std::abs(t_r.yaw()), M_PI, 1e-6));
}

TEST(Test_cslibs_math_3d, testTransformInverse)
{
    rng_t rng(-10.0, 10.0);
    const double x_0 = rng.get();
    const double y_0 = rng.get();
    const double z_0 = rng.get();
    const double roll_0  = cslibs_math::common::angle::normalize(rng.get());
    const double pitch_0 = cslibs_math::common::angle::normalize(rng.get());
    const double yaw_0   = cslibs_math::common::angle::normalize(rng.get());

    tf::Transform t_0_tf(tf::createQuaternionFromRPY(roll_0, pitch_0, yaw_0),
                       tf::Vector3(x_0, y_0, z_0));
    cslibs_math_3d::Transform3d t_0(cslibs_math_3d::Vector3d(x_0, y_0, z_0),
                                    cslibs_math_3d::Quaterniond(roll_0, pitch_0, yaw_0));

    cslibs_math_3d::Transform3d t_r = t_0.inverse();
    tf::Transform t_r_tf = t_0_tf.inverse();

    double t_r_tf_roll, t_r_tf_pitch, t_r_tf_yaw;
    tf::Matrix3x3(t_r_tf.getRotation()).getRPY(t_r_tf_roll,
                                               t_r_tf_pitch,
                                               t_r_tf_yaw);
    EXPECT_NEAR(t_r.tx(), t_r_tf.getOrigin().x(), 1e-5);
    EXPECT_NEAR(t_r.ty(), t_r_tf.getOrigin().y(), 1e-5);
    EXPECT_NEAR(t_r.tz(), t_r_tf.getOrigin().z(), 1e-5);
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_roll, t_r.roll(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_roll) + std::abs(t_r.roll()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_pitch, t_r.pitch(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_pitch + t_r.pitch()), M_PI, 1e-6));
    EXPECT_TRUE(cslibs_math::common::eq(t_r_tf_yaw, t_r.yaw(), 1e-6) ||
                cslibs_math::common::eq(std::abs(t_r_tf_yaw) + std::abs(t_r.yaw()), M_PI, 1e-6));
}

TEST(Test_cslibs_math_3d, testArrayAligned)
{
    cslibs_math_3d::Transform3d t1 = cslibs_math_3d::Transform3d::random();
    cslibs_math_3d::Transform3d t2 = cslibs_math_3d::Transform3d::random();
    cslibs_math_3d::Transform3d t3 = cslibs_math_3d::Transform3d::random();

    auto test_eq = [](const cslibs_math_3d::Transform3d &ta,
                      const cslibs_math_3d::Transform3d &tb)
    {
        EXPECT_EQ(ta.tx(), tb.tx());
        EXPECT_EQ(ta.ty(), tb.ty());
        EXPECT_EQ(ta.tz(), tb.tz());
        EXPECT_EQ(ta.yaw(), tb.yaw());
        EXPECT_EQ(ta.pitch(), tb.pitch());
        EXPECT_EQ(ta.roll(), tb.roll());
    };


    /// test init
    std::array<cslibs_math_3d::Transform3d, 3> ts = {{t1,t2,t3}};
    test_eq(ts[0], t1);
    test_eq(ts[1], t2);
    test_eq(ts[2], t3);

    /// test assignment
    t1 = cslibs_math_3d::Transform3d::random();
    t2 = cslibs_math_3d::Transform3d::random();
    t3 = cslibs_math_3d::Transform3d::random();

    ts[0] = t1;
    ts[1] = t2;
    ts[2] = t3;

    test_eq(ts[0], t1);
    test_eq(ts[1], t2);
    test_eq(ts[2], t3);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
