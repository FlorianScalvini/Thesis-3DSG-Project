//
// Created by ubuntu on 05/08/23.
//

#include "Rect.h"


Rect::Rect(const float &x, const float &y, const float &width, const float &height) :
        tlwh({x, y, width, height})
{
}

Rect::~Rect()
= default;

const float& Rect::x() const
{
    return tlwh[0];
}

const float& Rect::y() const
{
    return tlwh[1];
}

const float& Rect::width() const
{
    return tlwh[2];
}

const float& Rect::height() const
{
    return tlwh[3];
}

float& Rect::x()
{
    return tlwh[0];
}

float& Rect::y()
{
    return tlwh[1];
}

float& Rect::width()
{
    return tlwh[2];
}

float&Rect::height()
{
    return tlwh[3];
}

float Rect::cx() const {
    return (float)(tlwh[0]  + tlwh[2] / 2.0);
}

float Rect::cy() const{
    return (float)(tlwh[1]  + tlwh[3] / 2.0);
}

const float& Rect::tl_x() const
{
    return tlwh[0];
}

const float& Rect::tl_y() const
{
    return tlwh[1];
}

float Rect::br_x() const
{
    return tlwh[0] + tlwh[2];
}

float Rect::br_y() const
{
    return tlwh[1] + tlwh[3];
}

Tlbr Rect::getTlbr() const
{
    return {
            tlwh[0],
            tlwh[1],
            tlwh[0] + tlwh[2],
            tlwh[1] + tlwh[3],
    };
}

Xyah Rect::getXyah() const
{
    return {
            tlwh[0] + tlwh[2] / 2,
            tlwh[1] + tlwh[3] / 2,
            tlwh[2] / tlwh[3],
            tlwh[3],
    };
}

cv::Rect Rect::getRectCV() const {
    return cv::Rect((int)tlwh[0], (int)tlwh[1], (int)tlwh[2], (int)tlwh[3]);
}

float Rect::calcIoU(const Rect& bboxA, const Rect& bboxB)
{

    const float box_area = (bboxB.tlwh[2] + 1) * (bboxB.tlwh[3] + 1);
    const float iw = std::min(bboxA.tlwh[0] + bboxA.tlwh[2], bboxB.tlwh[0] + bboxB.tlwh[2]) - std::max(bboxA.tlwh[0], bboxB.tlwh[0]) + 1;
    float iou = 0;
    if (iw > 0)
    {
        const float ih = std::min(bboxA.tlwh[1] + bboxA.tlwh[3], bboxB.tlwh[1] + bboxB.tlwh[3]) - std::max(bboxA.tlwh[1], bboxB.tlwh[1]) + 1;
        if (ih > 0)
        {
            const float ua = (bboxA.tlwh[0] + bboxA.tlwh[2] - bboxA.tlwh[0] + 1) * (bboxA.tlwh[1] + bboxA.tlwh[3] - bboxA.tlwh[1] + 1) + box_area - iw * ih;
            iou = iw * ih / ua;
        }
    }
    return iou;
}

Rect Rect::generate_rect_by_tlbr(const Tlbr& tlbr)
{
    return Rect(tlbr[0], tlbr[1], tlbr[2] - tlbr[0], tlbr[3] - tlbr[1]);
}

Rect Rect::generate_rect_by_xyah(const Xyah& xyah)
{
    const auto width = xyah[2] * xyah[3];
    return Rect(xyah[0] - width / 2, xyah[1] - xyah[3] / 2, width, xyah[3]);
}
