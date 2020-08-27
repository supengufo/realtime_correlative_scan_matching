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
#include "parameters.h"
//#include <gperftools/profiler.h>
//STL
#include <unordered_map>
#include <iostream>
#include <ctime>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
using namespace std;
typedef boost::shared_ptr<tf::TransformBroadcaster> tf_TransformBroadcaster_Ptr;
void pubTF(tf_TransformBroadcaster_Ptr &tf_pub, const Pose2d &robot_pose) {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.getYaw());
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "laser";

    odom_trans.transform.translation.x = robot_pose.getX();
    odom_trans.transform.translation.y = robot_pose.getY();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
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
    int map_grid_sizes_x = 0;
    ph.getParam("map_grid_sizes_x", map_grid_sizes_x);
    map_params["map_grid_sizes_x"] = 1000;
    map_params["map_grid_sizes_y"] = 1000;
    map_params["map_ori_x"] = 500;
    map_params["map_ori_y"] = 500;
    map_params["resolution"] = 0.05;
    map_params["search_step_xy"] = 0.01;
    map_params["search_step_rad"] = 0.005;
    map_params["search_steps"] = 5;
    map_params["layers"] = 2;
    map_params["magnification"] = 2;
}

void getMapParams(const ros::NodeHandle &ph, MapParams &map_params) {
    map_params.map_grid_sizes_x = 1000;
    map_params.map_grid_sizes_y = 1000;
    map_params.map_ori_x = 500;
    map_params.map_ori_y = 500;
    map_params.resolution = 0.05;
    map_params.search_step_xy = 0.01;
    map_params.search_step_rad = 0.005;
    map_params.search_steps = 5;
    map_params.layers = 2;
    map_params.magnification = 2;
}

void pubGridMap(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
    auto grid_map_msg = mapper->getROSOccGridMapVector().front();
    map_pub.publish(grid_map_msg);
};

void pubGridMapLowResolution(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
    auto grid_map_msg = mapper->getROSOccGridMapVector().back();
    map_pub.publish(grid_map_msg);
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map", 1);
    ros::Publisher map_pub_low_resolution = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map_low", 1);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("mapping/odometry", 10);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("mapping/scan", 1);
    tf_TransformBroadcaster_Ptr odom_broadcaster(new tf::TransformBroadcaster());

    std::string bag_path,lidar_topic;
    ph.getParam("bag_file_path", bag_path);
    ph.getParam("lidar_topic", lidar_topic);
    cout << "bag_file_path: " << bag_path << endl;
    cout << "lidar_topic: " << lidar_topic << endl;

    MapParams map_params;
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
    Pose2d pose_estimate_cur, pose_estimate_pre;
//    ProfilerStart("CPUProfile");
    while (ros::ok() && bag_it != bag_view.end()) {
        std::cout << "---------------------------------------" << std::endl;
        clock_t time_start = clock();
        const auto scan = bag_it->instantiate<sensor_msgs::LaserScan>();
        scan->header.stamp = ros::Time::now();
        scan_pub.publish(scan);

        Pose2d pose_estimate = Pose2d(0, 0, 0);
        if (!init) {
            mapper->updateMultiSolutionMap(pose_estimate, scan);
            init = true;
        }
        else {
            mapper->RealTimeCorrelativeScanMatch(pose_estimate_pre, scan, pose_estimate);
            mapper->updateMultiSolutionMap(pose_estimate, scan);
        }
//        std::cout << "pose_estimate x y yaw : " << pose_estimate.getX() << " " << pose_estimate.getY() << " " << pose_estimate.getYaw() << std::endl;
        clock_t time_end = clock();
        cout<<"time use:"<<1000*(time_end-time_start)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
        pubOdom(odom_pub, pose_estimate);
        pubTF(odom_broadcaster, pose_estimate);
        pubGridMap(mapper, map_pub);
        pubGridMapLowResolution(mapper, map_pub_low_resolution);
        pose_estimate_cur = pose_estimate;
        pose_estimate_pre = pose_estimate_cur;
        ++bag_it;
        loop_rate.sleep();
    }
//    ProfilerStop();
}