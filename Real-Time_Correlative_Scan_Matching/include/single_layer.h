//
// Created by nrsl on 2020/8/27.
//

#ifndef SRC_SINGLE_LAYER_H
#define SRC_SINGLE_LAYER_H
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
#include "pose2d.h"
#include "parameters.h"

class SingleLayer {
public:
    typedef std::shared_ptr<SingleLayer> Ptr;
    explicit SingleLayer(std::unordered_map<std::string, double> &base_map_params);
    SingleLayer(MapParams &base_map_params);
    void initOccupancyGridMsg();
    void updateMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void updateMap(const Pose2d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void updateMap(const SingleLayer::Ptr &base_layer);
    void updateFreeGrids(std::vector<Eigen::Vector2d> &free_grids_coor);
    void updateOccGrids(Eigen::Vector2d &occ_grids_coor);
    void setGridLogValue(Eigen::Vector2d &coordinate, const float &log_value);
    float getGridLogValue(Eigen::Vector2d &coordinate);
    float getGridLogValue(Eigen::Vector2d &coordinate, float &unknown);
    float getGridProbValue(Eigen::Vector2d &coordinate);
    void updateMapFromBaseMap(const SingleLayer::Ptr &base_map, const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    bool checkCoordinateValid(Eigen::Vector2d &coordinate);

    double RealTimeCorrelativeScanMatch(const sensor_msgs::LaserScanPtr &point_cloud, Pose2d &pose_estimate);

    double RealTimeCorrelativeScanMatchCore(const sensor_msgs::LaserScanPtr &scan, const Pose2d &pose_estimate);
    nav_msgs::OccupancyGrid &getOccupancyGridMap();
    void getSearchParameters(const Pose2d &pose, SearchParameters &search_parameters);

private:
    nav_msgs::OccupancyGrid ros_grid_map_;
    std::vector<std::vector<Grid::Ptr>> grid_map_;
    int max_x_, max_y_;
    int min_x_, min_y_;
    int ori_x_, ori_y_;
//    std::unordered_map<std::string, double> this_map_params_;
    MapParams this_map_params_;
};


#endif //SRC_SINGLE_LAYER_H
