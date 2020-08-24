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
#include <iostream>
//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
using namespace std;
typedef boost::shared_ptr<tf::TransformBroadcaster> tf_TransformBroadcaster_Ptr;


void pubTF(tf_TransformBroadcaster_Ptr &tf_pub, const Pose2d &robot_pose) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.getYaw());
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "laser";

    odom_trans.transform.translation.x = robot_pose.getX();
    odom_trans.transform.translation.y = robot_pose.getY();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf_pub->sendTransform(odom_trans);
}
void pubOdom(const ros::Publisher &publisher, const Pose2d &robot_pose) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.getYaw());

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    odom.child_frame_id = "laser";

    odom.pose.pose.position.x = robot_pose.getX();
    odom.pose.pose.position.y = robot_pose.getY();
    odom.pose.pose.orientation = odom_quat;

    publisher.publish(odom);
}

void getMapParams(const ros::NodeHandle &ph, std::unordered_map<std::string, double> &map_params) {
//    ph.getParam("map_grid_sizes_x", map_params["map_grid_sizes_x"]);
//    ph.getParam("map_grid_sizes_y", map_params["map_grid_sizes_y"]);
//    ph.getParam("map_ori_x", map_params["map_ori_x"]);
//    ph.getParam("map_ori_y", map_params["map_ori_y"]);
//    ph.getParam("resolution", map_params["resolution"]);
//    ph.getParam("search_step_xy", map_params["search_step_xy"]);
//    ph.getParam("search_step_rad", map_params["search_step_rad"]);
//    ph.getParam("layers", map_params["layers"]);
//    ph.getParam("magnification", map_params["magnification"]);
    map_params["map_grid_sizes_x"] = 1000;
    map_params["map_grid_sizes_y"] = 1000;
    map_params["map_ori_x"] = 500;
    map_params["map_ori_y"] = 500;
    map_params["resolution"] = 0.05;
    map_params["search_step_xy"] = 0.01;
    map_params["search_step_rad"] = 0.005;
    map_params["layers"] = 1;
    map_params["magnification"] = 2;
}
void pubGridMap(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
    auto grid_map_msg = mapper->getROSOccGridMapVector().front();
//    cout << "pub map now ... " << grid_map_msg.header.stamp << endl;
    map_pub.publish(grid_map_msg);
};
void pubGridMapLowResolution(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
    auto grid_map_msg = mapper->getROSOccGridMapVector().back();
//    cout << "pub map now ... " << grid_map_msg.header.stamp << endl;
    map_pub.publish(grid_map_msg);
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "RTCSM");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map", 1);
    ros::Publisher map_pub_low_resolution = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map_low", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("mapping/odometry", 10);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("mapping/scan", 1);
    tf_TransformBroadcaster_Ptr odom_broadcaster(new tf::TransformBroadcaster());

    std::string bag_path("/home/nrsl/dataset/ogm/laser1_2018-07-14-17-31-41.bag");
    std::string lidar_topic("/scan");
    ph.getParam("bag_file_path", bag_path);
    ph.getParam("lidar_topic", lidar_topic);
    cout << "bag_file_path: " << bag_path << endl;
    cout << "lidar_topic: " << lidar_topic << endl;

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
        Pose2d pose_estimate = Pose2d(0, 0, 0);
        if (!init) {
//            pose_estimate = Eigen::Matrix3d::Identity();
//            pose_estimate = Pose2d(0, 0, 0);
            mapper->updateMultiSolutionMap(pose_estimate, scan);
//            cout<<"Init now ..."<<endl;
            init = true;
        }
        else {
//            double score = mapper->RealTimeCorrelativeScanMatch(pose_estimate_pre, scan, pose_estimate);
            mapper->RealTimeCorrelativeScanMatch(pose_estimate_pre, scan, pose_estimate);
//            cout<<"update map now ..."<<endl;
            mapper->updateMultiSolutionMap(pose_estimate, scan);
        }
        //
        //TODO:2.根据计算得到的pose，发布odom
        pubOdom(odom_pub, pose_estimate);
        pubTF(odom_broadcaster, pose_estimate);
        pubGridMap(mapper, map_pub);
        pubGridMapLowResolution(mapper, map_pub_low_resolution);
        pose_estimate_cur = pose_estimate;
        pose_estimate_pre = pose_estimate_cur;
//        ++bag_it;
        loop_rate.sleep();
    }

}