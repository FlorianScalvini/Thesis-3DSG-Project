//
// Created by ubuntu on 12/11/2021.
//

#ifndef TRACKING_TRACKOBJECTTRACKINGSORT_H
#define TRACKING_TRACKOBJECTTRACKINGSORT_H


#include "opencv2/tracking/kalman_filters.hpp"
#include "opencv2/tracking/tracking.hpp"
#include "opencv2/core/core.hpp"
#include <vector>
#include "Tracking/TrackingAlgorithm.h"
#include "Tracking/ObjectTracking.h"

class TrackObjectSort : public ObjectTracking {
public:
    explicit TrackObjectSort(const ObjectBoundingBox &obj);
    void activate(const size_t& frame_id, const size_t& track_id) override;
    void reActivate(const ObjectTracking &new_track, const size_t &frame_id, const int &new_track_id) override;
    void predict() override;
    void update(const ObjectTracking &new_track, const size_t &frame_id) override;
    void stateMatToxywh();
private:
    cv::KalmanFilter kF;
};


#endif //TRACKING_TRACKOBJECTTRACKINGSORT_H
