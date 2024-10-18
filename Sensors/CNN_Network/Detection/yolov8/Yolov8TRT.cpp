//
// From https://github.com/wang-xinyu/tensorrtx
//

#include "Yolov8TRT.h"


void Yolov8TRT::yoloV8ToRect(float *bbox) const {
    float r_w = _widthNetwork / static_cast<float>(_imgSize.width);
    float r_h = _heightNetwork / static_cast<float>(_imgSize.height);

    float ratio = (r_h > r_w) ? r_w : r_h;
    float padding_x = (r_h > r_w) ? 0 : (_widthNetwork - r_h * _imgSize.width) / 2;
    float padding_y = (r_h > r_w) ? (_heightNetwork - r_w * _imgSize.height) / 2 : 0;

    float l = (bbox[0] - padding_x) / ratio;
    float r = (bbox[2] - padding_x) / ratio;
    float t = (bbox[1] - padding_y) / ratio;
    float b = (bbox[3] - padding_y) / ratio;

    bbox[0] = std::max(0.0f, l);
    bbox[1] = std::max(0.0f, t);
    bbox[2] = std::min(static_cast<float>(_imgSize.width) - 1, r - l);
    bbox[3] = std::min(static_cast<float>(_imgSize.height) - 1, b - t);
}


std::map<unsigned int, std::vector<ObjectBoundingBox>> Yolov8TRT::getOutput() {
    int det_size = 6; // 4 Bbox  + Conf + Class id
    std::map<unsigned int, std::vector<ObjectBoundingBox>> output;
    for (int i = 0; i < _pProb[0] && i < 1000; i++) {
        if (_pProb[1 + det_size * i + 4] <= CONF_THRESH) continue;
        yoloV8ToRect(&_pProb[1 + det_size * i]);
        ObjectBoundingBox det = ObjectBoundingBox(&_pProb[1 + det_size * i]);
        if (output.count((int)det.class_id) == 0)
            output.emplace((unsigned int) det.class_id, std::vector<ObjectBoundingBox>());
        output[(int)det.class_id].push_back(det);
    }
    nms(output);
    return output;
}


