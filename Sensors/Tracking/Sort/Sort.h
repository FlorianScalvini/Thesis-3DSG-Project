//
// Created by ubuntu on 12/11/2021.
//

#ifndef SONIFIER_SORT_H
#define SONIFIER_SORT_H


#include <map>
#include <vector>
#include "opencv2/tracking/kalman_filters.hpp"
#include "opencv2/tracking/tracking.hpp"
#include "opencv2/core/core.hpp"
#include "CNN_Network/Detection/Detecteur.h"
#include "TrackObjectSort.h"
#include "Tracking/TrackingAlgorithm.h"

class Sort :  public TrackingAlgorithm {
public:
    explicit Sort(int max_age=10, float iou_threshold=0.5);
    std::vector<std::shared_ptr<ObjectTracking>> update(const std::vector<ObjectBoundingBox>& objects);
private:
    int max_age;
    int id;
    float iou_threshold;
    size_t frame_id_;
    size_t track_id_count_;
    std::vector<std::shared_ptr<ObjectTracking>> trackers;

};




#endif //SONIFIER_SORT_H
