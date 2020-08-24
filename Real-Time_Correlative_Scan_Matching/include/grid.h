//
// Created by nrsl on 2020/8/19.
//
#ifndef SRC_GRID_H
#define SRC_GRID_H
#include <memory>
#include <cmath>

template<typename T>
struct LogOdds {
    inline static T to(const T p) {
        return std::log(p/(1.0 - p));
    }
    inline static T from(const T l) {
        return 1.0/(1.0 + std::exp(-l));
    }
};

class Grid {
public:
    typedef std::shared_ptr<Grid> Ptr;
    Grid(float pro_free, float pro_occ);
    void updateFree();
    void updateOcc();
    void setGridLog(const float &value){
        log_prior_ = value;
    };
    const inline float& getGridLog() const {
        return log_prior_;
    };
    inline float getGridPro() const {
        return LogOdds<float>::from(log_prior_);
    }
private:
    float pro_prior_, log_prior_;
    float pro_occ_threshold_, pro_free_threshold_;
    float log_occ_threshold_, log_free_threshold_;
    const float log_free_, log_occ_;
};

#endif //SRC_GRID_H
