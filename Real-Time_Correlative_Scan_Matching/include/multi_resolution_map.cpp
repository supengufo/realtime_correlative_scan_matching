//
// Created by nrsl on 2020/8/19.
//
#include "multi_resolution_map.h"
MultipleResolutionMap::MultipleResolutionMap(MapParams &map_params) : base_map_params_(map_params) {
    SetupMultiResolutionMapParams();
}

void MultipleResolutionMap::SetupMultiResolutionMapParams() {
    for (int i = 0; i < base_map_params_.layers; ++i) {
        int each_layer_magnification = std::pow(base_map_params_.magnification, i);
        MapParams each_map_params;
        each_map_params.magnification = each_layer_magnification;
        each_map_params.map_grid_sizes_x = base_map_params_.map_grid_sizes_x/each_layer_magnification;
        each_map_params.map_grid_sizes_y = base_map_params_.map_grid_sizes_y/each_layer_magnification;
        each_map_params.map_ori_x = base_map_params_.map_ori_x/each_layer_magnification;
        each_map_params.map_ori_y = base_map_params_.map_ori_y/each_layer_magnification;
        each_map_params.resolution = base_map_params_.resolution*each_layer_magnification;
        each_map_params.search_step_xy = base_map_params_.search_step_xy*each_layer_magnification;
        each_map_params.search_step_rad = base_map_params_.search_step_rad*each_layer_magnification;
        each_map_params.search_steps = base_map_params_.search_steps;
        SingleLayer::Ptr SingleLayer(new class SingleLayer(each_map_params));
        multi_resolution_map_[i] = SingleLayer;
    }
}

void MultipleResolutionMap::UpdateMultiResolutionMap(const Pose2d &pose, const sensor_msgs::LaserScanPtr &point_cloud) {
    int layers = base_map_params_.layers;
    for (int i = 0; i < layers; ++i) {
        if (i == 0) {
            multi_resolution_map_[i]->UpdateMap(pose, point_cloud);
        }
        else {
            multi_resolution_map_[i]->UpdateMap(multi_resolution_map_[i - 1]);
        }
    }
}

SingleLayer::Ptr MultipleResolutionMap::get_idx_multi_resolution_map(int idx) {
    if (idx > get_map_layers()) {
        return nullptr;
    }
    return multi_resolution_map_[idx];
}
