//
// Created by nrsl on 2020/8/19.
//

#include "grid.h"
Grid::Grid(float pro_free, float pro_occ) : log_free_(LogOdds<float>::to(pro_free)), log_occ_(LogOdds<float>::to(pro_occ)) {
    pro_prior_ = 0.5;
    log_prior_ = LogOdds<float>::to(pro_prior_);
    pro_occ_threshold_ = 0.8;
    pro_free_threshold_ = 0.2;
    log_occ_threshold_ = LogOdds<float>::to(pro_occ_threshold_);
    log_free_threshold_ = LogOdds<float>::to(pro_free_threshold_);
}
void Grid::updateFree() {
    if(log_prior_<log_free_threshold_){
        return;
    }
    log_prior_ += log_free_;
}
void Grid::updateOcc() {
    if(log_prior_>log_occ_threshold_){
        return;
    }
    log_prior_ += log_occ_;
}
