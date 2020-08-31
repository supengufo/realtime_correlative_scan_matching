// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef POSE2D_H
#define POSE2D_H

#include <eigen3/Eigen/Core>
#include <vector>
#include <unordered_map>
#define PI 3.1415926

class Pose2d {
public:
    Pose2d() {
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
//        GenerateSinCosMap();
    }
//    static void GenerateSinCosMap();
    Pose2d(double x, double y, double theta) : x_(x), y_(y), theta_(theta) {
        double cos_value = cos(theta_);
        double sin_value = sin(theta_);
        R_ << cos_value, -sin_value,
                sin_value, cos_value;
    }

    Pose2d(const Pose2d &p) : x_(p.x_), y_(p.y_), theta_(p.theta_), R_(p.R_) {
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
//    static std::unordered_map<int,double> sin_map_;
//    static std::unordered_map<int,double> cos_map_;
};

//class Pose2d
//#pragma once
//#include <cmath>
//#include "Eigen/Core"
//#include "Eigen/Geometry"
//
//// Bring the 'difference' between two angles into [-pi; pi]
//template <typename T> T NormalizeAngleDifference(T difference) {
//    while (difference > M_PI) {
//        difference -= T(2. * M_PI);
//    }
//    while (difference < -M_PI) {
//        difference += T(2. * M_PI);
//    }
//    return difference;
//}
//
//template <typename FloatType> class Rigid2 {
//public:
//    using Vector = Eigen::Matrix<FloatType, 2, 1>;
//    using Rotation2D = Eigen::Rotation2D<FloatType>;
//
//    Rigid2()
//            : translation_(Vector::Identity()), rotation_(Rotation2D::Identity()) {}
//    Rigid2(double x, double y, double theta):translation_({x,y}),rotation_(theta){
//    }
//
//    // Rotation2D(double ): Construct a 2D counter clock wise rotation from the
//    // angle a in radian.
//    Rigid2(const Vector &translation, const Rotation2D &rotation)
//            : translation_(translation), rotation_(rotation) {}
//    //同上，给定旋转角度θ,double是弧度值。
//    Rigid2(const Vector &translation, const double rotation)
//            : translation_(translation), rotation_(rotation) {}
//
//    static Rigid2 Rotation(const double rotation) {
//        return Rigid2(Vector::Zero(), rotation);
//    }
//    static Rigid2 Rotation(const Rotation2D &rotation) {
//        return Rigid2(Vector::Zero(), rotation);
//    }
//    static Rigid2 Translation(const Vector &vector) {
//        return Rigid2(vector, Rotation2D::Identity());
//    }
//    static Rigid2<FloatType> Identity() {
//        return Rigid2<FloatType>(Vector::Zero(), Rotation2D::Identity());
//    }
//
//    template <typename OtherType> Rigid2<OtherType> cast() const {
//        return Rigid2<OtherType>(translation_.template cast<OtherType>(),
//                rotation_.template cast<OtherType>());
//    }
//
//    const Vector &translation() const { return translation_; }
//
//    Rotation2D rotation() const { return rotation_; }
//
//    double normalized_angle() const {
//        return NormalizeAngleDifference(rotation().angle());
//    }
//
//    Rigid2 inverse() const {
//        const Rotation2D rotation = rotation_.inverse();
//        const Vector translation = -(rotation * translation_);
//        return Rigid2(translation, rotation);
//    }
//
//    std::string DebugString() const {
//        std::string out;
//        out.append("{ t: [");
//        out.append(std::to_string(translation().x()));
//        out.append(", ");
//        out.append(std::to_string(translation().y()));
//        out.append("], r: [");
//        out.append(std::to_string(rotation().angle()));
//        out.append("] }");
//        return out;
//    }
//    inline double getX() const {
//        return translation_(0,0);
//    }
//    inline double getY() const {
//        return translation_(1,0);
//    }
//    inline double getYaw() const {
//        return rotation_.angle();
//    }
//private:
//    Vector translation_;
//    Rotation2D rotation_;
//
//
//};
//
//template <typename FloatType>
//Rigid2<FloatType> operator*(const Rigid2<FloatType> &lhs,
//                            const Rigid2<FloatType> &rhs) {
//    return Rigid2<FloatType>(lhs.rotation() * rhs.translation() +
//                             lhs.translation(),
//            lhs.rotation() * rhs.rotation());
//}
//
//template <typename FloatType>
//typename Rigid2<FloatType>::Vector
//operator*(const Rigid2<FloatType> &rigid,
//          const typename Rigid2<FloatType>::Vector &point) {
//    return rigid.rotation() * point + rigid.translation();
//}
//
//template <typename T>
//std::ostream &operator<<(std::ostream &os, const Rigid2<T> &rigid) {
//    os << rigid.DebugString();
//    return os;
//}
//
//using Rigid2d = Rigid2<double>;
//using Rigid2f = Rigid2<float>;
//using Pose2d = Rigid2d;
//using Pose2f = Rigid2f;
#endif

