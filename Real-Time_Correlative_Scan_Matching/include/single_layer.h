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
using namespace std;
using PointCloud = vector<Eigen::Vector2d>;

class SingleLayer {
public:
    typedef std::shared_ptr<SingleLayer> Ptr;
    SingleLayer(MapParams &base_map_params);
    void InitOccupancyGridMsg();
//    void updateMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void UpdateMap(const Pose2d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void UpdateMap(const SingleLayer::Ptr &base_layer);
    void UpdateFreeGrids(std::vector<Eigen::Vector2d> &free_grids_coor);
    void UpdateOccGrids(Eigen::Vector2d &occ_grids_coor);
//    float getGridLogValue(Eigen::Vector2d &coordinate, float &unknown);
//    float getGridProbValue(Eigen::Vector2d &coordinate);
//    void updateMapFromBaseMap(const SingleLayer::Ptr &base_map, const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    bool CheckCoordinateValid(Eigen::Vector2d &coordinate) const;
    static void GeneratePointCloud(const sensor_msgs::LaserScanPtr &scan, PointCloud &point_cloud);
    Pose2d RealTimeCorrelativeScanMatch(const sensor_msgs::LaserScanPtr &scan, map<double, Pose2d> &multi_candidates);
    double RealTimeCorrelativeScanMatch(const sensor_msgs::LaserScanPtr &scan, Pose2d &pose_estimate,map<double, Pose2d> &multi_candidates);;
    nav_msgs::OccupancyGrid &GetOccupancyGridMap();
    void GetSearchParameters(const Pose2d &pose, vector<SearchParameters> &candidates);
    float GetGridLogValue(Eigen::Vector2d &coordinate);
    void SetGridLogValue(Eigen::Vector2d &coordinate, const float &log_value);

private:
    double RealTimeCorrelativeScanMatchCore(const PointCloud &point_cloud);
    double RealTimeCorrelativeScanMatchCore(const sensor_msgs::LaserScanPtr &scan, const Pose2d &pose_estimate);

private:
    nav_msgs::OccupancyGrid ros_grid_map_;
    std::vector<std::vector<Grid::Ptr>> grid_map_;
    int max_x_{}, max_y_{};
    int min_x_{}, min_y_{};
    int ori_x_{}, ori_y_{};
    MapParams this_map_params_;
//    double RealTimeCorrelativeScanMatchCore(const sensor_msgs::LaserScanPtr &scan, const Pose2d &pose_estimate);
};


#endif //SRC_SINGLE_LAYER_H
