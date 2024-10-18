#ifndef OUTDOORNAV_OBJECTTRACKING_H
#define OUTDOORNAV_OBJECTTRACKING_H

#include <vector>
#include <opencv2/core.hpp>
#include "CNN_Network/Detection/Detecteur.h"

enum class STrackState {
    New = 0,
    Tracked = 1,
    Lost = 2,
    Removed = 3,
};

class ObjectTracking
{
public:
    ObjectTracking(const ObjectBoundingBox &obj);
    ~ObjectTracking();
    const STrackState& getSTrackState() const;
    const bool& isActivated() const;
    ObjectBoundingBox getObjectBoundingBox() const;
    const Rect& getRect() const;
    const Rect& getRectPrev() const;
    const float& getScore() const;
    const unsigned short& getDepth() const;
    const unsigned short& getPrevDepth() const;
    const unsigned int& getTrackId() const;
    const unsigned int& getFrameId() const;
    const unsigned int& getStartFrameId() const;
    const unsigned int& getTrackletLength() const;
    const unsigned int& getCountHighMvt() const;
    void increaseCountHighMvt();
    void resetCountHighMvt();

    virtual void activate(const size_t& frame_id, const size_t& track_id) = 0;
    virtual void reActivate(const ObjectTracking &new_track, const size_t &frame_id, const int &new_track_id) = 0;
    virtual void predict() = 0;
    virtual void update(const ObjectTracking &new_track, const size_t &frame_id) = 0;
    void markAsLost();
    void markAsRemoved();


protected:
    STrackState state_;
    ObjectBoundingBox obj;
    ObjectBoundingBox objPrev;
    bool is_activated_;
    unsigned int track_id_;
    unsigned int frame_id_;
    unsigned int start_frame_id_;
    unsigned int tracklet_len_;
    unsigned int countHighMvt;
};

#endif //OUTDOORNAV_OBJECTTRACKING_H