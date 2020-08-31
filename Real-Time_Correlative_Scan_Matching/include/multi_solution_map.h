//
// Created by nrsl on 2020/8/19.
//
#ifndef SRC_MULTI_SOLUTION_MAP_H
#define SRC_MULTI_SOLUTION_MAP_H
#include "vector"
#include "grid.h"
#include <eigen3/Eigen/Core>

#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

//3rd lib
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/algorithms/bresenham.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include "single_layer.h"
#include "pose2d.h"


class MultipleResolutionMap {
public:
    typedef std::shared_ptr<MultipleResolutionMap> Ptr;
    explicit MultipleResolutionMap(MapParams &map_params);
    void SetupMultiResolutionMapParams();
    void UpdateMultiResolutionMap(const Pose2d &pose, const sensor_msgs::LaserScanPtr &point_cloud);
    inline int get_map_layers()const {
        return int(base_map_params_.layers);
    }
    inline int get_magnification()const {
        return int(base_map_params_.magnification);
    }
    inline double get_search_step_xy()const {
        return base_map_params_.search_step_xy;
    }
    SingleLayer::Ptr get_idx_multi_resolution_map(int idx);
private:
    MapParams base_map_params_;
    std::unordered_map<int, SingleLayer::Ptr> multi_resolution_map_;
    cslibs_math_2d::algorithms::Bresenham::Ptr bresenham_algo_;
};

#endif //SRC_MULTI_SOLUTION_MAP_H
