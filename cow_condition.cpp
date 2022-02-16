
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

int cbcGetArea(double up_area[2], double down_area[2])
{
    return cbc.ComputerAS(up_area[0], up_area[1], down_area[0], down_area[1]);
}

int cbcGetInterestPoint(double point_num1[2], double point_num2[2], double point_num3[2], double point_num5[2], double point_num7[2])
{
    cv::Point2f p1, p2, p3, p5, p7;
    int re = cbc.GetInterestPoint(p1, p2, p3, p5, p7);
    point_num1[0] = p1.y;
    point_num1[1] = p1.x;
    point_num2[0] = p2.y;
    point_num2[1] = p2.x;
    point_num3[0] = p3.y;
    point_num3[1] = p3.x;
    point_num5[0] = p5.y;
    point_num5[1] = p5.x;
    point_num7[0] = p7.y;
    point_num7[1] = p7.x;
    return re;
}