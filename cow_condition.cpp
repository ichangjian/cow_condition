
#include "cbc.hpp"
#include "cow_condition.h"
#include "log.h"

CBC cbc;
int cbcSetConfig(double camera_hight, double fx, double fy, double cx, double cy, int save_flag)
{
    LOGD("camera_hight:%f", camera_hight);
    LOGD("fx:%f", fx);
    LOGD("fy:%f", fy);
    LOGD("cx:%f", cx);
    LOGD("cy:%f", cy);
    LOGD("save_flag:%d", save_flag);

    cbc.SetCamera(camera_hight, fx, fy, cx, cy);
    cbc.SetSaveFlag(save_flag);
    LOGD("finish SetConfig");
    return 0;
}

int cbcGetValue(int depth_image_width, int depth_image_height, unsigned char *depth_image_data, double *cow_H, double *cow_VL, double *cow_VR)
{
    LOGD("depth_image_width:%d", depth_image_width);
    LOGD("depth_image_height:%d", depth_image_height);

    cv::Mat depth(depth_image_height, depth_image_width, CV_16UC1, depth_image_data);

    return cbc.ComputerHVLR(depth, *cow_H, *cow_VL, *cow_VR);
}