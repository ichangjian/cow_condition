#include "cow_condition.h"
#include <opencv2/opencv.hpp>
#include <time.h>
#include <fstream>
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

    cv::Mat image = imread(argv[1], IMREAD_UNCHANGED);

    if (0)
    {
        std::ofstream obj;
        obj.open("dp.txt");
        for (size_t i = 0; i < image.rows; i++)
        {
            for (size_t j = 0; j < image.cols; j++)
            {
                obj << image.at<unsigned short>(i, j) << " ";
            }
            obj << "\n";
        }

        obj.close();

        return 0;
    }

    clock_t ST = clock();
    double camera_hight = 2.350;
    double fx = 424.977;
    double fy = 424.977;
    double cx = 420.337;
    double cy = 241.23;
    int save_flag = 1;
    cbcSetConfig(camera_hight, fx, fy, cx, cy, save_flag);
    double cow_H = 0;
    double cow_VL = 0;
    double cow_VR = 0;
    cout << "=================\n";
    cbcGetValue(image.cols, image.rows, image.data, &cow_H, &cow_VL, &cow_VR);
    std::cout << "time " << double(clock() - ST) / CLOCKS_PER_SEC << "\n";
    // getchar();
    cout << "cow_H" << cow_H << " cow_VL" << cow_VL << " cow_VR" << cow_VR << "\n";
    return 0;
}
