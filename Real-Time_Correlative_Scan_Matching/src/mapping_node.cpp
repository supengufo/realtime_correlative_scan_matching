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
#include <gperftools/profiler.h>
//STL
#include <unordered_map>
#include <iostream>
#include <ctime>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
using namespace std;
typedef boost::shared_ptr<tf::TransformBroadcaster> tf_TransformBroadcaster_Ptr;

class Runner {
private:
    ros::NodeHandle nh_, ph_;
    ros::Publisher map_pub_, map_pub_low_resolution_, odom_pub_, scan_pub_;
    tf_TransformBroadcaster_Ptr odom_broadcaster_;
    MapParams map_params_;
    rosbag::Bag bag_;
    std::string bag_path_, lidar_topic_;

public:
    Runner(ros::NodeHandle &nh, ros::NodeHandle &ph, std::string &bag_path, std::string &lidar_topic) : nh_(nh), ph_(ph), bag_path_(bag_path), lidar_topic_(lidar_topic) {
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map", 1);
        map_pub_low_resolution_ = nh.advertise<nav_msgs::OccupancyGrid>("mapping/grid_map_low", 1);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("mapping/odometry", 10);
        scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("mapping/scan", 1);
        odom_broadcaster_.reset(new tf::TransformBroadcaster());
        GetMapParams(ph_, map_params_);
        bag_.open(bag_path_, rosbag::bagmode::Read);
    }

    void Run() {
//        MapParams map_params;
        rosbag::View bag_view(bag_, rosbag::TopicQuery(lidar_topic_));
        auto bag_it = bag_view.begin();
        for (int i = 0; i < 700; ++i) {
            if (bag_it == bag_view.end()) break;
            ++bag_it;
        }

        ros::Rate loop_rate(10);
        Mapper::Ptr mapper(new Mapper(map_params_));
        bool init = false;
        Pose2d pose_estimate_cur, pose_estimate_pre;

        int count = 0;
        double total_time = 0;
        while (ros::ok() && bag_it != bag_view.end()) {
            std::cout << "---------------------------------------" << std::endl;
            clock_t time_start = clock();
            const auto scan = bag_it->instantiate<sensor_msgs::LaserScan>();
            scan->header.stamp = ros::Time::now();
            scan_pub_.publish(scan);
            Pose2d pose_estimate = Pose2d(0, 0, 0);
            if (!init) {
                mapper->UpdateMultiSolutionMap(pose_estimate, scan);
                init = true;
            }
            else {
                mapper->RealTimeCorrelativeScanMatch(pose_estimate_pre, scan, pose_estimate);
                mapper->UpdateMultiSolutionMap(pose_estimate, scan);
            }
            clock_t time_end = clock();
            auto uesd_time = 1000*(time_end - time_start)/(double) CLOCKS_PER_SEC;
            total_time += uesd_time;
            cout << "time use:" << uesd_time << "ms" << endl;
            PubOdom(odom_pub_, pose_estimate);
            PubTF(odom_broadcaster_, pose_estimate);
            PubGridMap(mapper, map_pub_);
            PubGridMapLowResolution(mapper, map_pub_low_resolution_);
            pose_estimate_cur = pose_estimate;
            pose_estimate_pre = pose_estimate_cur;
            ++bag_it;
            ++count;
//        if (count == 500) {
//            break;
//        }
            loop_rate.sleep();
        }
    }

    void PubTF(tf_TransformBroadcaster_Ptr &tf_pub, const Pose2d &robot_pose) {
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

    void PubOdom(const ros::Publisher &publisher, const Pose2d &robot_pose) {
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

    void GetMapParams(const ros::NodeHandle &ph, std::unordered_map<std::string, double> &map_params) {
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

    void GetMapParams(const ros::NodeHandle &ph, MapParams &map_params) {
//    ph.getParam("map_grid_sizes_x", map_params.map_grid_sizes_x);
//    ph.getParam("map_grid_sizes_y", map_params.map_grid_sizes_y);
//    ph.getParam("map_ori_x", map_params.map_ori_x);
//    ph.getParam("map_ori_y", map_params.map_ori_y);
//    ph.getParam("resolution", map_params.resolution);
//    ph.getParam("search_step_xy", map_params.search_step_xy);
//    ph.getParam("search_step_rad", map_params.search_step_rad);
//    ph.getParam("layers", map_params.layers);
//    ph.getParam("magnification", map_params.magnification);
//    cout << "map_grid_sizes_x: " << map_grid_sizes_x << endl;
        map_params.map_grid_sizes_x = 1000;
        map_params.map_grid_sizes_y = 1000;
        map_params.map_ori_x = 500;
        map_params.map_ori_y = 500;
        map_params.resolution = 0.05;
        map_params.search_step_xy = 0.01;
        map_params.search_step_rad = 0.01;
//    map_params.search_step_rad = M_PI*1./180;
        map_params.search_steps = 5;
        map_params.layers = 2;
        map_params.magnification = 2;
    }

    void PubGridMap(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
        auto grid_map_msg = mapper->getROSOccGridMapVector().front();
        map_pub.publish(grid_map_msg);
    };

    void PubGridMapLowResolution(const Mapper::Ptr &mapper, ros::Publisher &map_pub) {
        auto grid_map_msg = mapper->getROSOccGridMapVector().back();
        map_pub.publish(grid_map_msg);
    };
};

int main(int argc, char **argv) {
    ProfilerStart("/tmp/scan_matching_profile");
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    std::string bag_path("/home/nrsl/code/ogm_ws/src/data/2020-08-28-15-28-24.bag");
    std::string lidar_topic("/scan");
    Runner runner(nh, ph, bag_path, lidar_topic);
    runner.Run();
    ProfilerStop();
}