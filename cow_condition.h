#ifndef __COW_CONDITION__
#define __COW_CONDITION__

extern "C"
{
    /**
     * @brief 设置配置参数
     * 
     * @param camera_hight 深度相机距离牛站立平台的高度，单位米
     * @param fx 深度相机的内参fx
     * @param fy 深度相机的内参fy
     * @param cx 深度相机的内参cx
     * @param cy 深度相机的内参cy
     * @param save_flag 为 1 保存计算的过程图，为 0 不保存
     * @return int 运行成功返回 0
     */
    int cbcSetConfig(double camera_hight, double fx, double fy, double cx, double cy, int save_flag);

    /**
     * @brief 通过深度图获取高度、体积
     * 
     * @param depth_image_width 深度图像的宽
     * @param depth_image_height 深度图像的高
     * @param depth_image_data 深度图像内存地址,16无符号整形的数据
     * @param cow_H 返回牛背最高点到站立平台的高度，单位米
     * @param cow_VL 返回牛背左侧的体积，单位立方米
     * @param cow_VR 返回牛背右侧的体积，单位立方米
     * @return int 运行成功返回 0
     */
    int cbcGetValue(int depth_image_width, int depth_image_height, unsigned char *depth_image_data, double *cow_H, double *cow_VL, double *cow_VR);

    /**
     * @brief 获取牛背的面积
     * 
     * @param up_area    有两个值，第1个值为牛背上半部的表面积单位为平方米，第2个值为牛背上半部的图上的像素个数
     * @param down_area  有两个值，第1个值为牛背下半部的表面积单位为平方米，第2个值为牛背下半部的图上的像素个数
     * @return int 运行成功返回 0
     */
    int cbcGetArea(double up_area[2], double down_area[2]);

    /**
     * @brief 获取文档中说明的兴趣点在深度图上的坐标
     * 
     * @param num1 兴趣点1号点所在的行、列值
     * @param num2 兴趣点2号点所在的行、列值
     * @param num3 兴趣点3号点所在的行、列值
     * @param num5 兴趣点5号点所在的行、列值
     * @param num7 兴趣点7号点所在的行、列值
     * @return int 运行成功返回 0
     */
    int cbcGetInterestPoint(double point_num1[2], double point_num2[2], double point_num3[2], double point_num5[2], double point_num7[2]);
}

#endif
