//
// Created by nrsl on 2020/8/27.
//

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H
#include <vector>
#include <unordered_map>
#include "single_layer.h"
using namespace std;
//using PointCloud = vector<Eigen::Vector2d>;

struct MapParams {
    int map_grid_sizes_x;
    int map_grid_sizes_y;
    int map_ori_x;
    int map_ori_y;
    double resolution;
    double search_step_xy;
    double search_step_rad;
    int search_steps;
    int layers;
    int magnification;
    MapParams() = default;
};

class SearchParameters {
private:
    std::vector<Eigen::Vector2d> delta_xy{};
    double angle;
public:
    SearchParameters(const double yaw, double xy_step_length, int search_steps) : angle(yaw) {
        delta_xy.clear();
        for (int x = -search_steps; x <= search_steps; ++x) {
            for (int y = -search_steps; y < search_steps; ++y) {
                double x_coor = xy_step_length*double(x);
                double y_coor = xy_step_length*double(y);
                delta_xy.emplace_back(x_coor, y_coor);
            }
        }
    }

    SearchParameters() = default;
    double get_angle() const {
        return angle;
    }

    std::vector<Eigen::Vector2d> get_delta_xy() const {
        return delta_xy;
    }
};

#endif //SRC_PARAMETERS_H
