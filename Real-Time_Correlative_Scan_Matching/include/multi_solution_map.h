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
    MultipleResolutionMap(std::unordered_map<std::string, double> &map_params);
    MultipleResolutionMap(MapParams &map_params);
    void setupMultiResolutionMapParams();
    void updateMultiResolutionMap(const Pose2d &pose, const sensor_msgs::LaserScanPtr &point_cloud);
    void updateMultiResolutionMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanPtr &point_cloud);;
    inline int getMapLayers() {
        return int(base_map_params_.layers);
    }
    inline int getMapMagnification() {
        return int(base_map_params_.magnification);
    }
    inline float getSearchStepXY() {
        return base_map_params_.search_step_xy;
    }
    inline const SingleLayer::Ptr &getMultipleResolutionMaps(int idx) {
        return multi_resolution_map_[std::to_string(idx)];
    }
    const SingleLayer::Ptr getTargetLayerPtr(int idx);
private:
    void updateBaseLayer(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanPtr &point_cloud) {
    }
//    std::unordered_map<std::string, double> base_map_params_;
    MapParams base_map_params_;
    std::unordered_map<std::string, SingleLayer::Ptr> multi_resolution_map_;
    cslibs_math_2d::algorithms::Bresenham::Ptr bresenham_algo_;
};

#endif //SRC_MULTI_SOLUTION_MAP_H
