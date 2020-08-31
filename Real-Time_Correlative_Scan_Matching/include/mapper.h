//
// Created by nrsl on 2020/8/18.
//
#ifndef SRC_MAPPER_H
#define SRC_MAPPER_H
#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include "pose2d.h"
//Self
#include "grid.h"
#include "multi_solution_map.h"
#include "single_layer.h"

class Mapper {
public:
    struct Index2D {
        Index2D(int row, int col) : row(row), col(col) {}
        Index2D() = default;
        int row;
        int col;
    };

public:
    typedef std::shared_ptr<Mapper> Ptr;
    Mapper(const MapParams &map_params);
    std::vector<nav_msgs::OccupancyGrid> &getROSOccGridMapVector();
    void SaveMap(const std::string &img_dir, const std::string &cfg_dir); //
    void LoadMap(const std::string &img_dir, const std::string &cfg_dir); //
    void UpdateMultiSolutionMap(const Pose2d &pose_estimate, const sensor_msgs::LaserScanPtr &point_cloud);
    double RealTimeCorrelativeScanMatch(const Pose2d &initial_pose_estimate,
                                        const sensor_msgs::LaserScanPtr &point_cloud,
                                        Pose2d &pose_estimate);;
    int get_layers_count() const {
        return multiple_resolution_map_->get_map_layers();
    }
private:
    MultipleResolutionMap::Ptr multiple_resolution_map_;
    MapParams map_params_;
    std::vector<nav_msgs::OccupancyGrid> multi_occupancy_grid_vec_;
};


#endif //SRC_MAPPER_H
