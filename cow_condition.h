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
}

#endif
