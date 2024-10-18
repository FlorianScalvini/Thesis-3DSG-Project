//
// Created by ubuntu on 05/07/23.
//

#include "TrackObjectByteSort.h"



TrackObjectByteSort::TrackObjectByteSort(const ObjectBoundingBox& det) : ObjectTracking(det), kalman_filter_(){}

TrackObjectByteSort::~TrackObjectByteSort() = default;


void TrackObjectByteSort::activate(const size_t& frame_id, const size_t& track_id)
{
    kalman_filter_.initiate(mean_, covariance_, obj.bbox.getXyah());
    updateRect();
    state_ = STrackState::Tracked;
    if (frame_id == 1)
    {
        is_activated_ = true;
    }
    track_id_ = track_id;
    frame_id_ = frame_id;
    start_frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void TrackObjectByteSort::reActivate(const ObjectTracking &new_track, const size_t &frame_id, const int &new_track_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    obj.conf = new_track.getScore();
    if (0 <= new_track_id)
    {
        track_id_ = new_track_id;
    }
    frame_id_ = frame_id;
    tracklet_len_ = 0;
}

void TrackObjectByteSort::predict()
{
    if (state_ != STrackState::Tracked)
    {
        mean_[7] = 0;
    }
    kalman_filter_.predict(mean_, covariance_);
}

void TrackObjectByteSort::update(const ObjectTracking &new_track, const size_t &frame_id)
{
    kalman_filter_.update(mean_, covariance_, new_track.getRect().getXyah());

    updateRect();

    state_ = STrackState::Tracked;
    is_activated_ = true;
    obj.conf = new_track.getScore();
    frame_id_ = frame_id;
    tracklet_len_++;
}

void TrackObjectByteSort::updateRect()
{
    obj.bbox.width() = mean_[2] * mean_[3];
    obj.bbox.height() = mean_[3];
    obj.bbox.x() = mean_[0] - obj.bbox.width() / 2;
    obj.bbox.y() = mean_[1] - obj.bbox.height() / 2;
}