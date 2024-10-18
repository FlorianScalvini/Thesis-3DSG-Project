//
// Created by ubuntu on 12/11/2021.
//

#include "Sort.h"
#include <memory>
#include <utility>
#include <vector>
#include "TrackObjectSort.h"
#include <algorithm>
#include <list>
#include <numeric>
#include <random>

Sort::Sort(int max_age, float iou_threshold) : max_age(max_age), iou_threshold(iou_threshold), id(1){
    frame_id_ = 0;
    track_id_count_ = 0;
}

std::vector<std::shared_ptr<ObjectTracking>> Sort::update(const std::vector<ObjectBoundingBox>& objects)
{
    std::vector<std::shared_ptr<ObjectTracking>> output_stracks;
    std::vector<std::vector<int>> matches_idx;
    std::vector<int> unmatch_detection_idx, unmatch_track_idx;
    // Create new STracks using the result of object detection
    std::vector<std::shared_ptr<ObjectTracking>> det_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> det_low_stracks;
    for (const auto &object : objects)
    {
        const auto strack = std::make_shared<TrackObjectSort>(object);
        det_stracks.push_back(strack);
    }

    for(auto & tracker : trackers)
    {
        tracker->predict(); // Prediction Kalman
    }
    std::vector<Rect> det_rects;
    for (const auto & b_track : det_stracks)
    {
        det_rects.push_back(b_track->getRect());
    }

    std::vector<Rect> t_rects;
    for (const auto & b_track : trackers)
    {
        t_rects.push_back(b_track->getRect());
    }

    const auto ious = calcIous(t_rects, det_rects);

        linearAssignment(ious, (int)trackers.size(), (int)det_stracks.size(), iou_threshold,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

    for (const auto &match_idx : matches_idx)
    {
        const auto track = trackers[match_idx[0]];
        const auto det = det_stracks[match_idx[1]];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->update(*det, frame_id_);
        }
        else
        {
            track->reActivate(*det, frame_id_, -1);
        }
    }
    for (const auto &unmatch_idx : unmatch_track_idx)
    {
        if (trackers[unmatch_idx]->getSTrackState() == STrackState::Tracked)
        {
            trackers[unmatch_idx]->markAsLost();
        }
        else if(trackers[unmatch_idx]->getTrackId() + max_age < frame_id_)
        {
            trackers[unmatch_idx]->markAsRemoved();
        }
    }


    for (auto it = trackers.begin(); it != trackers.end(); ) {
        if ((*it)->getSTrackState() == STrackState::Removed) {
            it->reset();
            it = trackers.erase(it);
        } else {
            ++it;
        }
    }


    // Add new stracks
    for (const auto &unmatch_idx : unmatch_detection_idx)
    {
        const auto track = det_stracks[unmatch_idx];
        track_id_count_++;
        track->activate(frame_id_, track_id_count_);
        trackers.push_back(track);
        det_stracks[unmatch_idx].reset();
    }

    for (const auto &track : trackers)
    {
        if (track->isActivated())
        {
            output_stracks.push_back(track);
        }
    }
    frame_id_++;
    return output_stracks;



}



/*
cv::Mat Sort::iouMatrix(std::vector<TrackObjectSort> dets, std::vector<TrackObjectSort> trackers) {
    cv::Mat matrixIOU = cv::Mat((int)dets.size(), (int)trackers.size(), CV_32F);
    for(int i = 0; i < dets.size(); i++)
    {
        for(int j = 0; j < trackers.size(); j++)
        {
            float bboxTrackers [4];
            trackers[j].stateMatToxywh(bboxTrackers);
            matrixIOU.at<float>(i,j) = Rect::calcIoU(dets[i]., bboxTrackers);
        }
    }
    return matrixIOU;
}*/
