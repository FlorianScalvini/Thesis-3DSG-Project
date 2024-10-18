//
// Derived of https://github.com/wang-xinyu/tensorrtx
//

#ifndef OUTDOORNAV_YOLO_H
#define OUTDOORNAV_YOLO_H

namespace Yolo
{
    static constexpr int kNumAnchor = 3;
    static constexpr float IGNORE_THRESH = 0.1f;
    struct YoloKernel
    {
        int width;
        int height;
        float anchors[kNumAnchor * 2];
    };
    static constexpr int LOCATIONS = 4;
    struct alignas(float) Detection {
        //center_x center_y w h
        float bbox[LOCATIONS];
        float conf;  // bbox_conf * cls_conf
        float class_id;
    };
}

#endif //OUTDOORNAV_YOLO_H
