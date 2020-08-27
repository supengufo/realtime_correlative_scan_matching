//
// Created by nrsl on 2020/8/18.
//
#include "mapper.h"
#include "grid.h"

Mapper::Mapper(const std::unordered_map<std::string, double> &map_params) : map_params_(map_params) {
    multiple_resolution_map_.reset(new MultipleResolutionMap(map_params_));
    multi_occupancy_grid_vec_.resize(map_params_["layers"]);
}
//
//void Mapper::init(const Eigen::Matrix3d &pose_estimate,const sensor_msgs::LaserScanPtr &point_cloud) {
//    multiple_resolution_map_->initMultiResolutionMap(point_cloud);
//}

void Mapper::saveMap(const std::string &img_dir, const std::string &cfg_dir) {

}

void Mapper::loadMap(const std::string &img_dir, const std::string &cfg_dir) {

}

std::vector<nav_msgs::OccupancyGrid> &Mapper::getROSOccGridMapVector() {
    for (int i = 0; i < map_params_["layers"]; ++i) {
        multi_occupancy_grid_vec_[i] = multiple_resolution_map_->getMultipleResolutionMaps(i)->getOccupancyGridMap();//第0层是最高分辨率的
    }
    return multi_occupancy_grid_vec_;
}

void Mapper::updateMultiSolutionMap(const Eigen::Matrix3d &pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud) {
    multiple_resolution_map_->updateMultiResolutionMap(pose_estimate, point_cloud);
}

void Mapper::updateMultiSolutionMap(const Pose2d &pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud) {
    multiple_resolution_map_->updateMultiResolutionMap(pose_estimate, point_cloud);
}

double Mapper::RealTimeCorrelativeScanMatch(const Eigen::Matrix3d &initial_pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud, Eigen::Matrix3d &pose_estimate) {
    return 0;
}

double Mapper::RealTimeCorrelativeScanMatch(const Pose2d &initial_pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud, Pose2d &pose_estimate) {
    int layers = getLayersCount();
    pose_estimate = initial_pose_estimate;
    double score = 0;
    for (int i = layers - 1; i >= 0; --i) {
        auto map_layer = multiple_resolution_map_->getTargetLayerPtr(i);
        score = map_layer->RealTimeCorrelativeScanMatch(point_cloud, pose_estimate);
    }
    return score;
}

//double Mapper::RealTimeCorrelativeScanMatch(const Eigen::Matrix3d &initial_pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud, const MultipleResolutionMap::Ptr &multi_reso_probability_grid_map, Eigen::Matrix3d &&pose_estimate) {
////    int layers = multi_reso_probability_grid_map->
//}
