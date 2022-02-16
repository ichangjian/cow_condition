#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>

class CBC
{
private:
    cv::Mat image_depth_;
    cv::Mat image_cow_;
    cv::Mat mask_;

    int camera_hight_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    cv::Vec4f middle_line_;
    int cow_hight_;
    int save_flag_;
    std::vector<cv::Point2f> interest_points_;
    std::vector<double> VAS_U_;
    std::vector<double> VAS_D_;

    std::vector<std::vector<cv::Point>> FindBigestContour(cv::Mat src, int &index);
    double CowHeight(const cv::Mat &_image, cv::Point &_p);
    cv::Point GetEquilateralTriangle(cv::Point &_U, cv::Point &_D);
    void SplitUD(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, cv::Point &_U, cv::Point &_D);
    void GetLineABC(cv::Vec4f _line, double &A, double &B, double &C);
    cv::Vec4f VerticalLine(const cv::Vec4f &_line, const cv::Point &_p);
    void EraseRegion(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, cv::Point &_tail);
    void EraseRegion(const cv::Point &_U, const cv::Point &_M, const cv::Point &_D);
    void CameraCoor();
    void FillHole();
    void GetCameraXYZ(int _x, int _y, int _p, double &_X, double &_Y, double &_Z);
    std::vector<double> ComputerVAS(const cv::Mat &_image);
    void SplitImageUD(const cv::Vec4f &_line, const cv::Mat &_image, cv::Mat &_image_U, cv::Mat &_image_D);
    void drawLine(cv::Mat &_image, const cv::Vec4f &_line);
    cv::Point2f GetTailPosition(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, const cv::Point &_U, const cv::Point &_D);

public:
    CBC();
    ~CBC();
    void SetDepth(const cv::Mat _image);
    int SetCamera(double _camera_hight, double _fx, double _fy, double _cx, double _cy);
    int SetSaveFlag(int _flag);
    int ComputerHVLR(const cv::Mat &_image, double &_H, double &_VL, double &_VR);
    int ComputerAS(double &_A_U, double &_S_U, double &_A_D, double &_S_D);
    int GetInterestPoint(cv::Point2f &_num1, cv::Point2f &_num2, cv::Point2f &_num3, cv::Point2f &_num5, cv::Point2f &_num7);
};
