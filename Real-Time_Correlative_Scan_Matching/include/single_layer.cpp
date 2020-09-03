//
// Created by nrsl on 2020/8/27.
//
#include "single_layer.h"
/**
 *      Single layer
 */
SingleLayer::SingleLayer(MapParams &base_map_params)
        : this_map_params_(base_map_params) {
    std::vector<std::vector<Grid::Ptr>> tmp_map1(this_map_params_.map_grid_sizes_x, std::vector<Grid::Ptr>(this_map_params_.map_grid_sizes_y, nullptr));
    grid_map_.swap(tmp_map1);
    InitOccupancyGridMsg();
}

void SingleLayer::InitOccupancyGridMsg() {
    ros_grid_map_.header.frame_id = "map";
    ros_grid_map_.header.stamp = ros::Time::now();
    ros_grid_map_.info.width = this_map_params_.map_grid_sizes_x;
    ros_grid_map_.info.height = this_map_params_.map_grid_sizes_y;
    ros_grid_map_.info.resolution = this_map_params_.resolution;
    ros_grid_map_.info.origin.position.x = -this_map_params_.map_ori_x*this_map_params_.resolution;//这个地图的原点是以m为单位的
    ros_grid_map_.info.origin.position.y = -this_map_params_.map_ori_y*this_map_params_.resolution;
    max_x_ = ros_grid_map_.info.width/2.0; //cell size
    max_y_ = ros_grid_map_.info.height/2.0;
    ori_x_ = this_map_params_.map_ori_x;
    ori_y_ = this_map_params_.map_ori_y;
    min_x_ = -max_x_;
    min_y_ = -max_y_;
}

void SingleLayer::UpdateMap(const Pose2d &pose, const sensor_msgs::LaserScanConstPtr &scan) {
    const double &ang_min = scan->angle_min;
    const double &ang_inc = scan->angle_increment;
    const double &range_max = scan->range_max;
    const double &range_min = scan->range_min;
    const double &cell_size = this->this_map_params_.resolution;
    std::vector<std::pair<int, int>> free_cordinates;
    std::vector<std::pair<int, int>> occ_cordinates;

    int free_count = 0, occ_count = 0, continue_count = 0, multi_counts = 0;

    for (size_t i = 0; i < scan->ranges.size(); i++) {
        double R = scan->ranges.at(i);
        if (R > range_max || R < range_min) {
            ++continue_count;
            continue;
        }
        double angle = ang_inc*i + ang_min;
        double cangle = cos(angle);
        double sangle = sin(angle);
        Eigen::Vector2d p_lidar(R*cangle, R*sangle);
        Eigen::Vector2d p_odom = pose*p_lidar;
        int x_id_occ = p_odom.x() > 0 ? floor(p_odom.x()/cell_size) : ceil(p_odom.x()/cell_size);
        int y_id_occ = p_odom.y() > 0 ? floor(p_odom.y()/cell_size) : ceil(p_odom.y()/cell_size);
        ++occ_count;
        Eigen::Vector2d occ_point_coordinate = {x_id_occ, y_id_occ};
        UpdateOccGrids(occ_point_coordinate);
        std::vector<Eigen::Vector2d> free_point_coordinate;
        cslibs_math_2d::algorithms::Bresenham a0(cslibs_math_2d::Point2d(pose.getX(), pose.getY()), cslibs_math_2d::Point2d(p_odom.x(), p_odom.y()), cell_size);//TODO:直接使用cell_size_就不编译出错
        while (!a0.done()) {
            ++free_count;
            if (x_id_occ == a0.x() && y_id_occ == a0.y()) {
                ++multi_counts;
                UpdateOccGrids(occ_point_coordinate);
            }
            else {
                free_point_coordinate.emplace_back(a0.x(), a0.y());
            }
            ++a0;
        }
        UpdateFreeGrids(free_point_coordinate);
    }
}

void SingleLayer::UpdateMap(const SingleLayer::Ptr &base_layer) {
    int map_length_x = this_map_params_.map_grid_sizes_x;
    int map_length_y = this_map_params_.map_grid_sizes_y;
    int magnification = this_map_params_.magnification;
    for (int i = 0; i < map_length_x; ++i) {
        for (int j = 0; j < map_length_y; ++j) {
            int this_map_x = i*magnification;
            int this_map_y = j*magnification;
            float log_max = -1000; //
            Eigen::Vector2d target_coor(this_map_x, this_map_y);
            for (int k = 0; k < magnification; ++k) {
                for (int l = 0; l < magnification; ++l) {
                    Eigen::Vector2d tmp_coor(target_coor.x() + k, target_coor.y() + l);
                    log_max = std::max(log_max, base_layer->GetGridLogValue(tmp_coor));
                }
            }
            Eigen::Vector2d update_coor(i, j);
            SetGridLogValue(update_coor, log_max);
        }
    }
}
void SingleLayer::UpdateFreeGrids(std::vector<Eigen::Vector2d> &free_grids_coor) {
    for (auto &coor:free_grids_coor) {
        if (coor.x() > max_x_) {
            coor.x() = max_x_;
        }
        else if (coor.x() < min_x_) {
            coor.x() = min_x_;
        }
        if (coor.y() > max_y_) {
            coor.y() = max_y_;
        }
        else if (coor.y() < min_y_) {
            coor.y() = min_y_;
        }
        int update_x = coor.x() + ori_x_;
        int update_y = coor.y() + ori_y_;
        if (grid_map_[update_x][update_y] == nullptr) {
            Grid::Ptr grid;
            grid.reset(new Grid(0.4, 0.6));
            grid_map_[update_x][update_y] = grid;
            grid_map_[update_x][update_y]->UpdateFree();
            return;
        }
        grid_map_[coor.x() + ori_x_][coor.y() + ori_y_]->UpdateFree();
    }
}
void SingleLayer::UpdateOccGrids(Eigen::Vector2d &occ_grids_coor) {
    if (occ_grids_coor.x() > max_x_) {
        occ_grids_coor.x() = max_x_;
    }
    else if (occ_grids_coor.x() < min_x_) {
        occ_grids_coor.x() = min_x_;
    }
    if (occ_grids_coor.y() > max_y_) {
        occ_grids_coor.y() = max_y_;
    }
    else if (occ_grids_coor.y() < min_y_) {
        occ_grids_coor.y() = min_y_;
    }
    int update_x, update_y;
    update_x = occ_grids_coor.x() + ori_x_;
    update_y = occ_grids_coor.y() + ori_y_;

    if (grid_map_[update_x][update_y] == nullptr) {
        Grid::Ptr grid;
        grid.reset(new Grid(0.4, 0.6));
        grid_map_[update_x][update_y] = grid;
        grid_map_[update_x][update_y]->UpdateOcc();
        return;
    }
    grid_map_[update_x][update_y]->UpdateOcc();
}
nav_msgs::OccupancyGrid &SingleLayer::GetOccupancyGridMap() {
    int size_x = this_map_params_.map_grid_sizes_x;
    int size_y = this_map_params_.map_grid_sizes_y;
    ros_grid_map_.header.stamp = ros::Time::now();
    ros_grid_map_.data.clear();
    int occ_count = 0, free_count = 0, null_count = 0;
    for (int i = 0; i < size_x; ++i) {
        for (int j = 0; j < size_y; ++j) {
            if (grid_map_[i][j] == nullptr) {
                ros_grid_map_.data.push_back(-1);
                ++null_count;
                continue;
            }
            float prob = grid_map_[i][j]->getGridPro();
            if (std::abs(prob - 0.5) < 0.001) {
                ros_grid_map_.data.push_back(-1);
                ++free_count;
            }
            else {
                ros_grid_map_.data.push_back(prob*100);
                ++occ_count;
            }
        }
    }
    return ros_grid_map_;
}

void SingleLayer::SetGridLogValue(Eigen::Vector2d &coordinate, const float &log_value) {
    if (!CheckCoordinateValid(coordinate)) {
        return;
    }
    if (grid_map_[coordinate.x()][coordinate.y()] == nullptr) {
        Grid::Ptr grid;
        grid.reset(new Grid(0.4, 0.6));
        grid->setGridLog(log_value);
        grid_map_[coordinate.x()][coordinate.y()] = grid;
        return;
    }

    grid_map_[coordinate.x()][coordinate.y()]->setGridLog(log_value);
}

float SingleLayer::GetGridLogValue(Eigen::Vector2d &coordinate) {
    if (!CheckCoordinateValid(coordinate) || grid_map_[coordinate.x()][coordinate.y()] == nullptr) {
        return 0; //未知区域
    }
    return grid_map_[coordinate.x()][coordinate.y()]->getGridLog();
}

bool SingleLayer::CheckCoordinateValid(Eigen::Vector2d &coordinate) const {
    if (coordinate.x() >= 0 || coordinate.x() < this_map_params_.map_grid_sizes_x || coordinate.y() >= 0 || coordinate.y() < this_map_params_.map_grid_sizes_y) {
        return true;
    }
    else {
        return false;
    }
}

void SingleLayer::GetSearchParameters(const Pose2d &pose, vector<SearchParameters> &candidates) {
    double xy_step_length = this_map_params_.search_step_xy;
    double angle_step_length = this_map_params_.search_step_rad;
    int search_steps = this_map_params_.search_steps;
//    search_parameters.GenerateSearchParameters(pose, xy_step_length, angle_step_length, search_steps);
    candidates.clear();
    for (int angle = -search_steps; angle <= search_steps; ++angle) {
        double yaw = pose.getYaw() + angle*angle_step_length;
        SearchParameters searchParameters(yaw, xy_step_length, search_steps);
        candidates.push_back(searchParameters);
    }
}

double SingleLayer::RealTimeCorrelativeScanMatch(const sensor_msgs::LaserScanPtr &scan, Pose2d &pose_estimate) {
    cout << "map params:" << this_map_params_.resolution << endl;
    cout << "Input pose: " << pose_estimate.getX() << " " << pose_estimate.getY() << " " << pose_estimate.getYaw() << endl;
    vector<SearchParameters> search_parameters;
    GetSearchParameters(pose_estimate, search_parameters);
    Pose2d best_candidate;
    PointCloud point_cloud;
    GeneratePointCloud(scan, point_cloud);
    //TODO：这里需要先使用 pose_estimate 变换一下，这个基础上再加上一些扰动。
    point_cloud = pose_estimate*point_cloud;
    double max_score = 0;
    for (const auto &search_parameter:search_parameters) {
        Pose2d pose(search_parameter.get_angle(), 0, 0);
        auto point_cloud_tmp = pose*point_cloud;
        for (const auto &delta_xy:search_parameter.get_delta_xy()) {
            for (auto &p:point_cloud_tmp) {
                p += delta_xy;
            }
            double tmp_score = RealTimeCorrelativeScanMatchCore(point_cloud_tmp);
            if (tmp_score > max_score) {
                max_score = tmp_score;
                double yaw = pose_estimate.getYaw() + search_parameter.get_angle();
                double x = pose_estimate.getX() + delta_xy.x();
                double y = pose_estimate.getY() + delta_xy.y();
                best_candidate = Pose2d(yaw, x, y);
            }
        }
    }
    pose_estimate = best_candidate;
    cout << "Output pose: " << pose_estimate.getX() << " " << pose_estimate.getY() << " " << pose_estimate.getYaw() << endl;
    return max_score;
}

double SingleLayer::RealTimeCorrelativeScanMatchCore(const PointCloud &point_cloud) {
    const double &cell_size = this->this_map_params_.resolution;
    std::vector<std::pair<int, int>> free_cordinates;
    std::vector<std::pair<int, int>> occ_cordinates;
    double score = 0;
    for (auto &p_odom:point_cloud) {
//        Eigen::Vector2d p_odom = pose_estimate*p_lidar;
        int x_id_occ = p_odom.x() > 0 ? floor(p_odom.x()/cell_size) : ceil(p_odom.x()/cell_size);
        int y_id_occ = p_odom.y() > 0 ? floor(p_odom.y()/cell_size) : ceil(p_odom.y()/cell_size);
        if (x_id_occ > min_x_ && x_id_occ < max_x_ && y_id_occ > min_y_ && y_id_occ < max_y_) {
            x_id_occ = x_id_occ + ori_x_;
            y_id_occ = y_id_occ + ori_y_;
            if (grid_map_[x_id_occ][y_id_occ] != nullptr) {
                score += grid_map_[x_id_occ][y_id_occ]->getGridLog();
            }
        }
    }
    return score;
}

double SingleLayer::RealTimeCorrelativeScanMatchCore(const sensor_msgs::LaserScanPtr &scan, const Pose2d &pose_estimate) {
    const double &cell_size = this->this_map_params_.resolution;
    std::vector<std::pair<int, int>> free_cordinates;
    std::vector<std::pair<int, int>> occ_cordinates;
    double score = 0;
    PointCloud point_cloud;
    GeneratePointCloud(scan, point_cloud);
    for (auto &p_lidar:point_cloud) {
        Eigen::Vector2d p_odom = pose_estimate*p_lidar;
        int x_id_occ = p_odom.x() > 0 ? floor(p_odom.x()/cell_size) : ceil(p_odom.x()/cell_size);
        int y_id_occ = p_odom.y() > 0 ? floor(p_odom.y()/cell_size) : ceil(p_odom.y()/cell_size);
        if (x_id_occ > min_x_ && x_id_occ < max_x_ && y_id_occ > min_y_ && y_id_occ < max_y_) {
            x_id_occ = x_id_occ + ori_x_;
            y_id_occ = y_id_occ + ori_y_;
            if (grid_map_[x_id_occ][y_id_occ] != nullptr) {
                score += grid_map_[x_id_occ][y_id_occ]->getGridLog();
            }
        }
    }
    return score;
}


void SingleLayer::GeneratePointCloud(const sensor_msgs::LaserScanPtr &scan, PointCloud &point_cloud) {
    const double &ang_min = scan->angle_min;
    const double &ang_inc = scan->angle_increment;
    const double &range_max = scan->range_max;
    const double &range_min = scan->range_min;
    point_cloud.clear();
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        double R = scan->ranges.at(i);
        if (R > range_max || R < range_min) {
            continue;
        }
        double angle = ang_inc*i + ang_min;
        double cangle = cos(angle);
        double sangle = sin(angle);
        Eigen::Vector2d p_lidar(R*cangle, R*sangle);
        point_cloud.push_back(p_lidar);
    }
}
