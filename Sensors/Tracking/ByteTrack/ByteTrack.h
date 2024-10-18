//
// Created by ubuntu on 05/07/23.
//

#ifndef OUTDOORNAV_BYTETRACK_H
#define OUTDOORNAV_BYTETRACK_H

#include <vector>
#include "TrackObjectByteSort.h"
#include "CNN_Network/Detection/Detecteur.h"
#include "../TrackingAlgorithm.h"



class ByteTrack : public TrackingAlgorithm{

public:
    ByteTrack(const int& frame_rate = 30,
              const int& track_buffer = 30,
              const float& track_thresh = 0.5,
              const float& high_thresh = 0.6,
              const float& match_thresh = 0.8);
    ~ByteTrack();

    std::vector<std::shared_ptr<ObjectTracking>> update(const std::vector<ObjectBoundingBox>& objects);




private:
    const float track_thresh_;
    const float high_thresh_;
    const float match_thresh_;
    const size_t max_time_lost_;

    size_t frame_id_;
    size_t track_id_count_;

    std::vector<std::shared_ptr<ObjectTracking>> tracked_stracks_;
    std::vector<std::shared_ptr<ObjectTracking>> lost_stracks_;
};


#endif //OUTDOORNAV_BYTETRACK_H
