//
// Created by ubuntu on 05/07/23.
//

#include "ByteTrack.h"
#include "Tracking/lapjv.h"
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>




ByteTrack::ByteTrack(const int& frame_rate, const int& track_buffer, const float& track_thresh, const float& high_thresh, const float& match_thresh) :
        track_thresh_(track_thresh), high_thresh_(high_thresh), match_thresh_(match_thresh), max_time_lost_(static_cast<size_t>(frame_rate / 30.0 * track_buffer)),
        frame_id_(0), track_id_count_(0) {}

ByteTrack::~ByteTrack()= default;

std::vector<std::shared_ptr<ObjectTracking>> ByteTrack::update(const std::vector<ObjectBoundingBox>& objects)
{
    ////////////////// Step 1: Get detections //////////////////
    frame_id_++;
    // Create new STracks using the result of object detection
    std::vector<std::shared_ptr<ObjectTracking>> det_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> det_low_stracks;

    for (const auto &object : objects)
    {
        const auto strack = std::make_shared<TrackObjectByteSort>(object);
        if (object.conf >= track_thresh_)
        {
            det_stracks.push_back(strack);
        }
        else
        {
            det_low_stracks.push_back(strack);
        }
    }


    // Create lists of existing STrack
    std::vector<std::shared_ptr<ObjectTracking>> active_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> non_active_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> strack_pool;

    for (const auto& tracked_strack : tracked_stracks_)
    {
        if (!tracked_strack->isActivated())
        {
            non_active_stracks.push_back(tracked_strack);
        }
        else
        {
            active_stracks.push_back(tracked_strack);
        }
    }

    strack_pool = jointStracks(active_stracks, lost_stracks_);

    // Predict current pose by KF
    for (auto &strack : strack_pool)
    {
        strack->predict();
    }

    ////////////////// Step 2: First association, with IoU //////////////////
    std::vector<std::shared_ptr<ObjectTracking>> current_tracked_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> remain_tracked_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> remain_det_stracks;
    std::vector<std::shared_ptr<ObjectTracking>> refind_stracks;

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_detection_idx, unmatch_track_idx;

        const auto dists = calcIouDistance(strack_pool, det_stracks);
        linearAssignment(dists, (int)strack_pool.size(), (int)det_stracks.size(), match_thresh_,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = strack_pool[match_idx[0]];
            const auto det = det_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_, -1);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            remain_det_stracks.push_back(det_stracks[unmatch_idx]);
        }

        for (const auto &unmatch_idx : unmatch_track_idx)
        {
            if (strack_pool[unmatch_idx]->getSTrackState() == STrackState::Tracked)
            {
                remain_tracked_stracks.push_back(strack_pool[unmatch_idx]);
            }
        }
    }

    ////////////////// Step 3: Second association, using low score dets //////////////////
    std::vector<std::shared_ptr<ObjectTracking>> current_lost_stracks;

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_track_idx, unmatch_detection_idx;

        const auto dists = calcIouDistance(remain_tracked_stracks, det_low_stracks);
        linearAssignment(dists, (int) remain_tracked_stracks.size(), (int) det_low_stracks.size(), 0.5,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = remain_tracked_stracks[match_idx[0]];
            const auto det = det_low_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_, -1);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_track : unmatch_track_idx)
        {
            const auto track = remain_tracked_stracks[unmatch_track];
            if (track->getSTrackState() != STrackState::Lost)
            {
                track->markAsLost();
                current_lost_stracks.push_back(track);
            }
        }
    }

    ////////////////// Step 4: Init new stracks //////////////////
    std::vector<std::shared_ptr<ObjectTracking>> current_removed_stracks;

    {
        std::vector<int> unmatch_detection_idx;
        std::vector<int> unmatch_unconfirmed_idx;
        std::vector<std::vector<int>> matches_idx;

        // Deal with unconfirmed tracks, usually tracks with only one beginning frame
        const auto dists = calcIouDistance(non_active_stracks, remain_det_stracks);
        linearAssignment(dists, (int) non_active_stracks.size(), (int) remain_det_stracks.size(), 0.7,
                         matches_idx, unmatch_unconfirmed_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            non_active_stracks[match_idx[0]]->update(*remain_det_stracks[match_idx[1]], frame_id_);
            current_tracked_stracks.push_back(non_active_stracks[match_idx[0]]);
        }

        for (const auto &unmatch_idx : unmatch_unconfirmed_idx)
        {
            const auto track = non_active_stracks[unmatch_idx];
            track->markAsRemoved();
            current_removed_stracks.push_back(track);
        }

        // Add new stracks
        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            const auto track = remain_det_stracks[unmatch_idx];
            if (track->getScore() < high_thresh_)
            {
                continue;
            }
            track_id_count_++;
            track->activate(frame_id_, track_id_count_);
            current_tracked_stracks.push_back(track);
        }
    }

    ////////////////// Step 5: Update state //////////////////
    for (const auto &lost_strack : lost_stracks_)
    {
        if (frame_id_ - lost_strack->getFrameId() > max_time_lost_)
        {
            lost_strack->markAsRemoved();
            current_removed_stracks.push_back(lost_strack);
        }
    }


    tracked_stracks_ = jointStracks(current_tracked_stracks, refind_stracks);
    lost_stracks_ = jointStracks(subStracks(lost_stracks_, tracked_stracks_), current_lost_stracks);


    std::vector<std::shared_ptr<ObjectTracking>> tracked_stracks_out, lost_stracks_out;
    removeDuplicateStracks(tracked_stracks_, lost_stracks_, tracked_stracks_out, lost_stracks_out);
    tracked_stracks_ = tracked_stracks_out;
    lost_stracks_ = lost_stracks_out;

    for (auto it = current_removed_stracks.begin(); it != current_removed_stracks.end(); ) {
        if ((*it)->getSTrackState() == STrackState::Lost) {
            it->reset();
            it = current_removed_stracks.erase(it);
        } else {
            ++it;
        }
    }
    std::vector<std::shared_ptr<ObjectTracking>> output_stracks;
    for (const auto &track : tracked_stracks_) {
        if (track->isActivated()) {
            output_stracks.push_back(track);
        }
    }
}
