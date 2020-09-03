//
// Created by nrsl on 2020/8/18.
//
#include "mapper.h"
#include "grid.h"

Mapper::Mapper(const MapParams &map_params) : map_params_(map_params) {
    multiple_resolution_map_.reset(new MultipleResolutionMap(map_params_));
    multi_occupancy_grid_vec_.resize(map_params_.layers);
}

void Mapper::SaveMap(const std::string &img_dir, const std::string &cfg_dir) {
    /**/
}

void Mapper::LoadMap(const std::string &img_dir, const std::string &cfg_dir) {
    /**/
}

std::vector<nav_msgs::OccupancyGrid> &Mapper::getROSOccGridMapVector() {
    for (int i = 0; i < map_params_.layers; ++i) {
        multi_occupancy_grid_vec_[i] = multiple_resolution_map_->get_idx_multi_resolution_map(i)->GetOccupancyGridMap();//第0层是最高分辨率的
    }
    return multi_occupancy_grid_vec_;
}

void Mapper::UpdateMultiSolutionMap(const Pose2d &pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud) {
    multiple_resolution_map_->UpdateMultiResolutionMap(pose_estimate, point_cloud);
}

double Mapper::RealTimeCorrelativeScanMatch(const Pose2d &initial_pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud, Pose2d &pose_estimate) {
    int layers = get_layers_count();
    pose_estimate = initial_pose_estimate;
    double score = 0;
    auto low_resolution_map_layer = multiple_resolution_map_->get_idx_multi_resolution_map(1);
    map<double, Pose2d> multi_candidates;
    score = low_resolution_map_layer->RealTimeCorrelativeScanMatch(point_cloud, pose_estimate, multi_candidates);
    for (const auto &i:multi_candidates) {
        cout << i.first << " " << i.second.getX() << " " << i.second.getY() << " " << i.second.getYaw() << endl;
    }
    auto high_resolution_map_layer = multiple_resolution_map_->get_idx_multi_resolution_map(0);

    Pose2d final_pose = high_resolution_map_layer->RealTimeCorrelativeScanMatch(point_cloud, multi_candidates);

    cout << "final_pose:" << final_pose.getX() << " " << final_pose.getY() << " " << final_pose.getYaw() << endl;

    pose_estimate = final_pose;
    return score;
}
