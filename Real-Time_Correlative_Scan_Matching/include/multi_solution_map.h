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
#include "pose2d.h"

class SingleLayer {
public:
    typedef std::shared_ptr<SingleLayer> Ptr;
    explicit SingleLayer(std::unordered_map<std::string, double> &base_map_params);
    void initOccupancyGridMsg();
    void updateMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void updateMap(const Pose2d &pose, const sensor_msgs::LaserScanConstPtr &scan);
    void updateMap(const SingleLayer::Ptr &base_layer);
    void updateFreeGrids( std::vector<Eigen::Vector2d> &free_grids_coor);
    void updateOccGrids( Eigen::Vector2d &occ_grids_coor);
    void setGridLogValue( Eigen::Vector2d &coordinate,const float &log_value);
    float getGridLogValue( Eigen::Vector2d &coordinate);
    float getGridLogValue( Eigen::Vector2d &coordinate,float& unknown);
    float getGridProbValue(Eigen::Vector2d &coordinate);

    void updateMapFromBaseMap(const SingleLayer::Ptr &base_map, const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan);

    bool checkCoordinateValid(Eigen::Vector2d &coordinate){
        if(coordinate.x()>=0||coordinate.x()<this_map_params_["map_grid_sizes_x"]||coordinate.y()>=0||coordinate.y()<this_map_params_["map_grid_sizes_y"]){
            return true;
        }else{
            return false;
        }
    }

//    void initScan(const sensor_msgs::LaserScanConstPtr &scan);
    nav_msgs::OccupancyGrid &getOccupancyGridMap();
private:
    nav_msgs::OccupancyGrid ros_grid_map_;
    std::vector<std::vector<Grid::Ptr>> grid_map_;
    int max_x_, max_y_;
    int min_x_, min_y_;
    int ori_x_,ori_y_;
    std::unordered_map<std::string, double> this_map_params_;
};

class MultipleResolutionMap {
public:
    typedef std::shared_ptr<MultipleResolutionMap> Ptr;
    explicit MultipleResolutionMap(std::unordered_map<std::string, double> &map_params);
    void setupMultiResolutionMapParams();
    void updateMultiResolutionMap(const Pose2d &pose, const sensor_msgs::LaserScanPtr &point_cloud);
    void updateMultiResolutionMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanPtr &point_cloud);;

    inline int getMapLayers() {
        return int(base_map_params_["layers"]);
    }

    inline int getMapMagnification() {
        return int(base_map_params_["magnification"]);
    }
    inline float getSearchStepXY() {
        return base_map_params_["search_step_xy"];
    }
    inline const SingleLayer::Ptr &getMultipleResolutionMaps(int idx) {
        return multi_resolution_map_[std::to_string(idx)];
    }
private:
    void updateBaseLayer(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanPtr &point_cloud) {

    }
    std::unordered_map<std::string, double> base_map_params_;
    std::unordered_map<std::string, SingleLayer::Ptr> multi_resolution_map_;
    cslibs_math_2d::algorithms::Bresenham::Ptr bresenham_algo_;
};


#endif //SRC_MULTI_SOLUTION_MAP_H
