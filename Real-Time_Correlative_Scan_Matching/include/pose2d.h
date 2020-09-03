// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef POSE2D_H
#define POSE2D_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <unordered_map>
#define PI 3.1415926
using namespace std;
using PointCloud = vector<Eigen::Vector2d>;

class Pose2d {
public:
    Pose2d() {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }
    Pose2d(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {
        double cos_value = cos(theta_);
        double sin_value = sin(theta_);
        R_ << cos_value, -sin_value,
                sin_value, cos_value;
    }

    Pose2d(const Pose2d &p) : x_(p.x_), y_(p.y_), theta_(p.theta_), R_(p.R_) {
    }

    PointCloud operator*(const PointCloud &point_cloud) {
        PointCloud point_cloud_return;
        for (const auto &point:point_cloud) {
            Eigen::Vector2d p = (*this)*point;
            point_cloud_return.push_back(p);
        }
        return point_cloud_return;
    }
    const Pose2d operator*(const Pose2d &p2) {
        Eigen::Vector2d pt2(p2.x_, p2.y_);
        Eigen::Vector2d pt = R_*pt2 + Eigen::Vector2d(x_, y_);
        double theta;
        theta = theta_ + p2.theta_;
        NormAngle(theta);
        return Pose2d(pt(0), pt(1), theta);
    }

    Eigen::Vector2d operator*(const Eigen::Vector2d &p) const {
        Eigen::Vector2d t(x_, y_);
        return R_*p + t;
    }

    Pose2d inv() {
        double x = -(cos(theta_)*x_ + sin(theta_)*y_);
        double y = -(-sin(theta_)*x_ + cos(theta_)*y_);
        double theta = -theta_;
        return Pose2d(x, y, theta);
    }

    void setXYandYaw(double x, double y, double theta) {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }

    void NormAngle(double &angle) {
        if (angle >= PI)
            angle -= 2.0*PI;
        if (angle < -PI)
            angle += 2.0*PI;
    }
    inline double getX() const {
        return x_;
    }
    inline double getY() const {
        return y_;
    }
    inline double getYaw() const {
        return theta_;
    }
private:
    double x_, y_, theta_;
    Eigen::Matrix2d R_;
};

#endif

