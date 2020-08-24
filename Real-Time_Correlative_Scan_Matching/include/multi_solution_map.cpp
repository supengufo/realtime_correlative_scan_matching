//
// Created by nrsl on 2020/8/19.
//

#include "multi_solution_map.h"
//multiple layer
MultipleResolutionMap::MultipleResolutionMap(std::unordered_map<std::string, double> &map_params) :
        base_map_params_(map_params) {
    setupMultiResolutionMapParams();
}
void MultipleResolutionMap::setupMultiResolutionMapParams() {
    for (int i = 0; i < base_map_params_["layers"]; ++i) {

        int each_layer_magnification = std::pow(base_map_params_["magnification"], i);
        std::unordered_map<std::string, double> each_map_params = base_map_params_;
        each_map_params["magnification"] = each_layer_magnification;
        each_map_params["map_grid_sizes_x"] = base_map_params_["map_grid_sizes_x"]/each_layer_magnification;
        each_map_params["map_grid_sizes_y"] = base_map_params_["map_grid_sizes_y"]/each_layer_magnification;
        each_map_params["map_ori_x"] = base_map_params_["map_ori_x"]/each_layer_magnification;//这里应该是m为单位吧？
        each_map_params["map_ori_y"] = base_map_params_["map_ori_y"]/each_layer_magnification;
        each_map_params["resolution"] = base_map_params_["resolution"]*each_layer_magnification;
        each_map_params["search_step_xy"] = base_map_params_["search_step_xy"]*each_layer_magnification;

        SingleLayer::Ptr SingleLayer(new class SingleLayer(each_map_params));
        multi_resolution_map_[std::to_string(i)] = SingleLayer;
    }
}
void MultipleResolutionMap::updateMultiResolutionMap(const Pose2d &pose, const sensor_msgs::LaserScanPtr &point_cloud) {
    int layers = base_map_params_["layers"];
    for (int i = 0; i < layers; ++i) {
        //TODO:0 . 先把点云的坐标转换到全局坐标系下。
        //TODO:1 . bresenham生成occ的坐标和free的坐标,把所有的坐标传进去进行更新。（base_map）
        if (i == 0) {
            multi_resolution_map_[std::to_string(i)]->updateMap(pose, point_cloud);
        }
        else {
            multi_resolution_map_[std::to_string(i)]->updateMap(multi_resolution_map_[std::to_string(i - 1)]);
        }
    }
}
void MultipleResolutionMap::updateMultiResolutionMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanPtr &point_cloud) {

}

/**
 *      Single layer
 */
SingleLayer::SingleLayer(std::unordered_map<std::string, double> &base_map_params)
        : this_map_params_(base_map_params) {
    std::vector<std::vector<Grid::Ptr>> tmp_map1(this_map_params_["map_grid_sizes_x"], std::vector<Grid::Ptr>(this_map_params_["map_grid_sizes_y"], nullptr));
    grid_map_.swap(tmp_map1);
    initOccupancyGridMsg();
}
void SingleLayer::initOccupancyGridMsg() {
    ros_grid_map_.header.frame_id = "map";
    ros_grid_map_.header.stamp = ros::Time::now();
    ros_grid_map_.info.width = this_map_params_["map_grid_sizes_x"];
    ros_grid_map_.info.height = this_map_params_["map_grid_sizes_y"];
    ros_grid_map_.info.resolution = this_map_params_["resolution"];
    ros_grid_map_.info.origin.position.x = -this_map_params_["map_ori_x"]*this_map_params_["resolution"];//这个地图的原点是以m为单位的
    ros_grid_map_.info.origin.position.y = -this_map_params_["map_ori_y"]*this_map_params_["resolution"];
    max_x_ = ros_grid_map_.info.width/2.0; //cell size
    max_y_ = ros_grid_map_.info.height/2.0;
    ori_x_ = this_map_params_["map_ori_x"];
    ori_y_ = this_map_params_["map_ori_y"];
    min_x_ = -max_x_;
    min_y_ = -max_y_;
}
void SingleLayer::updateMap(const Pose2d &pose, const sensor_msgs::LaserScanConstPtr &scan) {
    /* 获取激光的信息 */
    const double &ang_min = scan->angle_min;
//    const double &ang_max = scan->angle_max;
    const double &ang_inc = scan->angle_increment;
    const double &range_max = scan->range_max;
    const double &range_min = scan->range_min;

    /* 设置遍历的步长，沿着一条激光线遍历 */
    const double &cell_size = this->this_map_params_["resolution"];
    std::vector<std::pair<int, int>> free_cordinates;
    std::vector<std::pair<int, int>> occ_cordinates;

    //1.先把scan根据pose转换到当前坐标下。
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
        updateOccGrids(occ_point_coordinate);
        std::vector<Eigen::Vector2d> free_point_coordinate;
        //2.再根据bresenham生成free点和occ点。
//        std::cout<<occ_point_coordinate.x()<<" "<<occ_point_coordinate.y()<<std::endl;

        cslibs_math_2d::algorithms::Bresenham a0(cslibs_math_2d::Point2d(pose.getX(), pose.getY()), cslibs_math_2d::Point2d(p_odom.x(), p_odom.y()), cell_size);//TODO:直接使用cell_size_就不编译出错
        while (!a0.done()) {
            ++free_count;
//            std::cout<<a0.x()<<","<<a0.y()<<std::endl;
            if (x_id_occ == a0.x() && y_id_occ == a0.y()) {
                ++multi_counts;
                updateOccGrids(occ_point_coordinate);
            }
            else {
                free_point_coordinate.emplace_back(a0.x(), a0.y());
            }
            ++a0;
        }
        updateFreeGrids(free_point_coordinate);
    }
    std::cout << "all points:" << scan->ranges.size() << " occ: " << occ_count << " free:" << free_count << " continue:" << continue_count << " multi:" << multi_counts << std::endl;
}
void SingleLayer::updateMap(const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan) {
}
void SingleLayer::updateMapFromBaseMap(const SingleLayer::Ptr &base_map, const Eigen::Matrix3d &pose, const sensor_msgs::LaserScanConstPtr &scan) {
}
void SingleLayer::updateMap(const SingleLayer::Ptr &base_layer) {

}
void SingleLayer::updateFreeGrids(std::vector<Eigen::Vector2d> &free_grids_coor) {
//    std::cout<<"max_x_:"<<max_x_<<" max_y_"<<max_y_<<std::endl;
//    std::cout<<"min_x_:"<<min_x_<<" min_y_"<<min_y_<<std::endl;
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
//        std::cout<<"x: "<<coor.x() + ori_x_<<" y:"<<coor.y() + ori_y_<<std::endl;
        int update_x = coor.x() + ori_x_;
        int update_y = coor.y() + ori_y_;
        if (grid_map_[update_x][update_y] == nullptr) {
            Grid::Ptr grid;
            grid.reset(new Grid(0.4, 0.6));
            grid_map_[update_x][update_y] = grid;
            grid_map_[update_x][update_y]->updateFree();
            return;
        }
        grid_map_[coor.x() + ori_x_][coor.y() + ori_y_]->updateFree();
    }
}
void SingleLayer::updateOccGrids(Eigen::Vector2d &occ_grids_coor) {
    //避免越界
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
//    std::cout<<"x: "<<occ_grids_coor.x() + ori_x_<<" y:"<<occ_grids_coor.y() + ori_y_<<std::endl;
//    std::cout<<"ori_x_: "<<ori_x_<<" ori_y:"<<ori_y_<<std::endl;
    int update_x = occ_grids_coor.x() + ori_x_;
    int update_y = occ_grids_coor.y() + ori_y_;

    if (grid_map_[update_x][update_y] == nullptr) {
        Grid::Ptr grid;
        grid.reset(new Grid(0.4, 0.6));
        grid_map_[update_x][update_y] = grid;
        grid_map_[update_x][update_y]->updateOcc();
        return;
    }

    grid_map_[update_x][update_y]->updateOcc();
}
nav_msgs::OccupancyGrid &SingleLayer::getOccupancyGridMap() {
    int size_x = this_map_params_["map_grid_sizes_x"];
    int size_y = this_map_params_["map_grid_sizes_y"];
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
//    std::cout << "ros_grid_map_.data size:" << ros_grid_map_.data.size() << std::endl;
//    std::cout << "free size:" << free_count << " occ_count:" << occ_count << " null count:" << null_count << std::endl;
    return ros_grid_map_;
}

