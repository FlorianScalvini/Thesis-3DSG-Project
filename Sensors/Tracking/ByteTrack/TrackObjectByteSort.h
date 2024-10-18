//
// Created by ubuntu on 05/07/23.
//

#ifndef OUTDOORNAV_TRACKOBJECTBYTESORT_H
#define OUTDOORNAV_TRACKOBJECTBYTESORT_H

#include "CNN_Network/Detection/Detecteur.h"
#include "Tracking/ObjectTracking.h"
#include "KalmanFilter.h"


class TrackObjectByteSort : public ObjectTracking {
public:
    explicit TrackObjectByteSort(const ObjectBoundingBox &obj);
    ~TrackObjectByteSort();
    void activate(const size_t& frame_id, const size_t& track_id) override;
    void reActivate(const ObjectTracking &new_track, const size_t &frame_id, const int &new_track_id) override;
    void predict() override;
    void update(const ObjectTracking &new_track, const size_t &frame_id) override;

private:
    KalmanFilter kalman_filter_;
    Eigen::Matrix<float, 1, 8, Eigen::RowMajor>  mean_;
    Eigen::Matrix<float, 8, 8, Eigen::RowMajor> covariance_;
    void updateRect();
};

#endif //OUTDOORNAV_TRACKOBJECTBYTESORT_H
