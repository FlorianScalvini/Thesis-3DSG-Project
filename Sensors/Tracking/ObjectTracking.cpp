#include "ObjectTracking.h"


ObjectTracking::ObjectTracking(const ObjectBoundingBox& det) : obj(det), is_activated_(false), state_(STrackState::New),
                                                               track_id_(0), frame_id_(0), start_frame_id_(0), tracklet_len_(0){
    objPrev = ObjectBoundingBox();
}

ObjectTracking::~ObjectTracking() = default;


const Rect &ObjectTracking::getRect() const {
    return obj.bbox;
}

ObjectBoundingBox ObjectTracking::getObjectBoundingBox() const{
    return obj;
}

const Rect &ObjectTracking::getRectPrev() const {
    return objPrev.bbox;
}

const STrackState& ObjectTracking::getSTrackState() const
{
    return state_;
}

const bool& ObjectTracking::isActivated() const
{
    return is_activated_;
}
const float& ObjectTracking::getScore() const
{
    return obj.conf;
}

const unsigned short &ObjectTracking::getDepth() const {
    return obj.depth;
}

const unsigned short &ObjectTracking::getPrevDepth() const {
    return objPrev.depth;
}

const unsigned int& ObjectTracking::getTrackId() const
{
    return track_id_;
}

const unsigned int& ObjectTracking::getFrameId() const
{
    return frame_id_;
}

const unsigned int& ObjectTracking::getStartFrameId() const
{
    return start_frame_id_;
}

const unsigned int& ObjectTracking::getTrackletLength() const
{
    return tracklet_len_;
}

const unsigned int &ObjectTracking::getCountHighMvt() const {
    return countHighMvt;
}

void ObjectTracking::increaseCountHighMvt() {
    countHighMvt++;
}

void ObjectTracking::resetCountHighMvt() {
    countHighMvt = 0;
}

void ObjectTracking::markAsLost()
{
    state_ = STrackState::Lost;
}

void ObjectTracking::markAsRemoved()
{
    state_ = STrackState::Removed;
}
