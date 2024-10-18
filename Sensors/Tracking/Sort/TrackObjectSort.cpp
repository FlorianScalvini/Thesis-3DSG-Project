//
// Created by ubuntu on 12/11/2021.
//

#include "TrackObjectSort.h"

#define DIM_X 7
#define DIM_Z 4


TrackObjectSort::TrackObjectSort(const ObjectBoundingBox& det) : ObjectTracking(det){
    track_id_ = 0;
    /*
    float stateValue[DIM_X][DIM_X] = {
            {1, 0, 0, 0, 1, 0, 0},
            {0, 1, 0, 0, 0, 1, 0},
            {0, 0, 1, 0, 0, 0, 1},
            {0, 0, 0, 1, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0},
            {0, 0, 0, 0, 0, 1, 0},
            {0, 0, 0, 0, 0, 0, 1},
    };
    float measureValue[DIM_Z][DIM_X] = {
            {1, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0}
    };
    */

    this->kF = cv::KalmanFilter(DIM_X, DIM_Z, 0, CV_32F);
    kF.transitionMatrix.at<float>(0,4) = 1.0;
    kF.transitionMatrix.at<float>(1,5) = 1.0;
    kF.transitionMatrix.at<float>(2,6) = 1.0;

    kF.measurementMatrix.at<float>(0) = 1.0;
    kF.measurementMatrix.at<float>(8) = 1.0;
    kF.measurementMatrix.at<float>(16) = 1.0;
    kF.measurementMatrix.at<float>(24) = 1.0;

    kF.measurementNoiseCov.at<float>(2,2) *= 10.0;
    kF.measurementNoiseCov.at<float>(3,3) *= 10.0;

    kF.processNoiseCov.at<float>(DIM_X - 1, DIM_X - 1) *= 0.01;
    cv::setIdentity(kF.errorCovPost);
    cv::setIdentity(kF.errorCovPre);
    for(int i=4; i<DIM_X;i++)
    {
        kF.errorCovPost.at<float>(i,i) *= 1000.0;
        kF.processNoiseCov.at<float>(i,i) *= 0.01;
    }
    kF.errorCovPost *= 10.0;
    kF.statePost.at<float>(0) = obj.bbox.x();
    kF.statePost.at<float>(1) = obj.bbox.y();
    kF.statePost.at<float>(2) = obj.bbox.width() * obj.bbox.height();
    kF.statePost.at<float>(3) = obj.bbox.width() / obj.bbox.height();
}

void TrackObjectSort::predict() {
    if(kF.statePost.at<float>(6) + kF.statePost.at<float>(2) <= 0)
    {
        kF.statePost.at<float>(6) = 0.0;
    }
    kF.predict();
    stateMatToxywh();
}

void TrackObjectSort::update(const ObjectTracking &new_track, const size_t &frame_id) {
    state_ = STrackState::Tracked;
    is_activated_ = true;
    obj.conf = new_track.getScore();
    frame_id_ = frame_id;
    tracklet_len_++;
    objPrev = obj;
    obj.bbox = new_track.getRect();
    obj.depth = new_track.getDepth();
    cv::Mat bboxMat = cv::Mat(4,1,CV_32F);
    bboxMat.at<float>(0) = obj.bbox.x();
    bboxMat.at<float>(1) = obj.bbox.y();
    bboxMat.at<float>(2) = obj.bbox.width() * obj.bbox.height();
    bboxMat.at<float>(3) = obj.bbox.width() / obj.bbox.height();
    kF.correct(bboxMat);
}

void TrackObjectSort::reActivate(const ObjectTracking &new_track, const size_t &frame_id, const int &new_track_id)
{
    cv::Mat bboxMat = cv::Mat(4,1,CV_32F);
    bboxMat.at<float>(0) = obj.bbox.x();
    bboxMat.at<float>(1) = obj.bbox.y();
    bboxMat.at<float>(2) = obj.bbox.width() * obj.bbox.height();
    bboxMat.at<float>(3) = obj.bbox.width() / obj.bbox.height();
    kF.correct(bboxMat);
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

void TrackObjectSort::stateMatToxywh()
{
    obj.bbox.tlwh[0] = kF.statePost.at<float>(0);
    obj.bbox.tlwh[1] = kF.statePost.at<float>(1);
    obj.bbox.tlwh[2] = (float)sqrt((double)kF.statePost.at<float>(2) * kF.statePost.at<float>(3));
    obj.bbox.tlwh[3] = kF.statePost.at<float>(2) / obj.bbox.tlwh[2];
}

void TrackObjectSort::activate(const size_t &frame_id, const size_t &track_id) {
    this->frame_id_ = frame_id;
    this->state_ = STrackState::Tracked;
    this->track_id_ = track_id;
}