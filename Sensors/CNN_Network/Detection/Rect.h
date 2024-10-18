//
// Created by Florian on 05/08/23.
//

#ifndef OUTDOORNAV_RECT_H
#define OUTDOORNAV_RECT_H



#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using Tlwh = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;
using Tlbr = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;
using Xyah = Eigen::Matrix<float, 1, 4, Eigen::RowMajor>;

class Rect
{
public:
    Tlwh tlwh;
    Rect() = default;
    Rect(const float &x, const float &y, const float &width, const float &height);

    ~Rect();

    const float &x() const;
    const float &y() const;
    const float &width() const;
    const float &height() const;

    float &x();
    float &y();
    float &width();
    float &height();

    float cx() const;
    float cy() const;

    const float &tl_x() const;
    const float &tl_y() const;
    float br_x() const;
    float br_y() const;

    Tlbr getTlbr() const;
    Xyah getXyah() const;
    cv::Rect getRectCV() const;
    static float calcIoU(const Rect& bboxA, const Rect& bboxB);

    static Rect generate_rect_by_tlbr(const Tlbr& tlbr);

    static Rect generate_rect_by_xyah(const Xyah& xyah);
};


#endif //OUTDOORNAV_RECT_H
