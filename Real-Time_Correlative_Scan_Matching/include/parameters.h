//
// Created by nrsl on 2020/8/27.
//

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H
#include <vector>
#include <unordered_map>

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
    MapParams() {
    }
};

struct SearchParameters {
    int min_x, min_y, max_x, max_y;
    double min_rad, max_rad;
    std::vector<Pose2d> candidates;
    void GenerateSearchParameters(const Pose2d &pose, double xy_step_length, double angle_step_length, int search_steps) {
        for (int angle = -search_steps; angle <= search_steps; ++angle) {
            for (int x = -search_steps; x <= search_steps; ++x) {
                for (int y = -search_steps; y < search_steps; ++y) {
                    double x_coor = pose.getX() + xy_step_length*double(x);
                    double y_coor = pose.getY() + xy_step_length*double(y);
                    double yaw_coor = pose.getYaw() + angle_step_length*angle;
                    Pose2d pose_tmp(x_coor, y_coor, yaw_coor);
                    candidates.push_back(pose_tmp);
                }
            }
        }
    }
    SearchParameters() = default;
    SearchParameters(const Pose2d &pose, double xy_step_length, double angle_step_length, int search_steps) {
        GenerateSearchParameters(pose, xy_step_length, angle_step_length, search_steps);
    }

};

#endif //SRC_PARAMETERS_H
