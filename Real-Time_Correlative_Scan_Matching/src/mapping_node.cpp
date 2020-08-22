//
// Created by nrsl on 2020/8/18.
//
#include <ros/ros.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/QuaternionStamped.h>
//self
#include "mapper.h"
#include "pose2d.h"

//STL
#include <unordered_map>
//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

typedef boost::shared_ptr<tf::TransformBroadcaster> tf_TransformBroadcaster_Ptr;


void pubTF(const tf_TransformBroadcaster_Ptr &tf_pub, const double &x, const double &y, const double &theta) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf_pub->sendTransform(odom_trans);
}
void pubOdom(const ros::Publisher &publisher, double x, double y, double theta) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = odom_quat;

    publisher.publish(odom);
}

void getMapParams(const ros::NodeHandle &ph, std::unordered_map<std::string, double> &map_params) {
    ph.getParam("map_grid_sizes_x", map_params["map_grid_sizes_x"]);
    ph.getParam("map_grid_sizes_y", map_params["map_grid_sizes_y"]);
    ph.getParam("map_ori_x", map_params["map_ori_x"]);
    ph.getParam("map_ori_y", map_params["map_ori_y"]);
    ph.getParam("resolution", map_params["resolution"]);
    ph.getParam("search_step_xy", map_params["search_step_xy"]);
    ph.getParam("search_step_rad", map_params["search_step_rad"]);
    ph.getParam("layers", map_params["layers"]);
    ph.getParam("magnification", map_params["magnification"]);
}
void pubGridMap(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
    auto grid_map_msg = mapper->getROSOccGridMapVector().front();
    map_pub.publish(grid_map_msg);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RTCSM");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("mapping/odometry", 10);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("mapping/scan", 1);
    tf_TransformBroadcaster_Ptr odom_broadcaster(new tf::TransformBroadcaster());

    std::string bag_path, lidar_topic;
    ph.getParam("bag_file_path", bag_path);
    ph.getParam("lidar_topic", lidar_topic);
    std::unordered_map<std::string, double> map_params;
    getMapParams(ph, map_params);

    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    rosbag::View bag_view(bag, rosbag::TopicQuery(lidar_topic));
    auto bag_it = bag_view.begin();

    for (int i = 0; i < 700; ++i) {
        if (bag_it == bag_view.end()) break;
        ++bag_it;
    }

    ros::Rate loop_rate(10);
    Mapper::Ptr mapper(new Mapper(map_params));
    bool init = false;
//    Eigen::Matrix3d pose_estimate_cur, pose_estimate_pre;
    Pose2d pose_estimate_cur, pose_estimate_pre;

    while (ros::ok() && bag_it != bag_view.end()) {
        const auto scan = bag_it->instantiate<sensor_msgs::LaserScan>();
        scan_pub.publish(scan);

//        Eigen::Matrix3d pose_estimate;
        Pose2d pose_estimate;
        if (!init) {
//            pose_estimate = Eigen::Matrix3d::Identity();
            pose_estimate = Pose2d(0, 0, 0);
            mapper->updateMultiSolutionMap(pose_estimate, scan);
            init = true;
        }
        else {
            double score = mapper->RealTimeCorrelativeScanMatch(pose_estimate_pre, scan, pose_estimate);
            mapper->updateMultiSolutionMap(pose_estimate, scan);
        }
        //
        //TODO:2.根据计算得到的pose，发布odom
//        pubOdom(odom_pub, x, y, theta);
//        pubTF(odom_broadcaster, x, y, theta);
        pubGridMap(mapper, map_pub);
        pose_estimate_cur = pose_estimate;
        pose_estimate_pre = pose_estimate_cur;
        ++bag_it;
        loop_rate.sleep();
    }

}