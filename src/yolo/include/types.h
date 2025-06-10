#ifndef TYPES_H
#define TYPES_H

#include <string>
#include<vector>

struct Detection
{

    // x1, y1, x2, y2 左上角点和右下角点
    float bbox[4];  // bbox both before and after scale
    float conf;
    int classId;
    float kpts[4 * 2];  // key points: 4*2=8, before scale to original image
    std::vector<std::vector<float>> vKpts;  // key points after scale: {{x, y, visible}, {x, y, visible}, {x, y, visible}, ...}
};


#endif  // TYPES_H
