#include "log.h"
#include "cbc.hpp"
using namespace cv;
// #include <opencv2/ximgproc.hpp>
#define SHOW_IMG
cv::Mat IMG;

CBC::CBC()
{
    camera_hight_ = 2350;
    fx_ = 424.977;
    fy_ = 424.977;
    cx_ = 420.337;
    cy_ = 241.23;
    save_flag_ = 0;
}

CBC::~CBC()
{
}

void CBC::SetDepth(const cv::Mat _image)
{
    image_depth_ = _image;

    image_cow_ = camera_hight_ - image_depth_;
    // cv::erode(image_cow_, image_cow_, cv::Mat(5, 5, CV_8UC1, cv::Scalar(1)));

    cv::Mat bw = (image_cow_ > 1200) & (image_cow_ < 2000);
    int index;
    std::vector<std::vector<cv::Point>> contours = FindBigestContour(bw, index);

    cv::Mat cow_roi = Mat::zeros(image_cow_.rows, image_cow_.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, index, cv::Scalar(1), -1);

    mask_ = Mat::zeros(image_cow_.rows, image_cow_.cols, CV_8UC1);
    cv::drawContours(mask_, contours, index, cv::Scalar(1), -1);

    image_cow_ = image_cow_.mul(cow_roi);
    image_depth_ = image_depth_.mul(cow_roi);

    image_cow_.convertTo(IMG, CV_8UC1, 0.1);

    cv::Point U, D;
    cv::Vec4f line;
    cv::fitLine(contours[index], line, DIST_L2, 0, 0.01, 0.01);
    middle_line_ = line;
    SplitUD(contours[index], line, U, D);

    cv::Point mid_point = GetEquilateralTriangle(U, D);
    cv::circle(IMG, mid_point, 5, cv::Scalar(0), -1);
    imshow("img", IMG);
    waitKey();
    line = VerticalLine(line, mid_point);
    cv::Point tail_point;
    EraseRegion(U, mid_point, D);
    cv::Mat roi;
    mask_.convertTo(roi, CV_16UC1);
    // image_depth_ = image_depth_.mul(roi > 5);
    // EraseRegion(contours[index], line, tail_point);
    cv::multiply(image_depth_, mask_, image_depth_, 0, CV_16UC1);

    image_depth_.convertTo(IMG, CV_8UC1, 0.1);
    FillHole();
    CameraCoor();
    imshow("img1", IMG);
    waitKey();
    cv::Point top_point;
    cow_hight_ = CowHeight(camera_hight_ - image_depth_, top_point);
    std::cout << cow_hight_ << "\n";
    image_depth_.convertTo(IMG, CV_8UC1, 0.1);
    imshow("img1", IMG);
    waitKey();
    std::vector<cv::Point> mids;
    mids.push_back(top_point);
    mids.push_back(mid_point);
    // mids.push_back((U + D) / 2);
    cv::fitLine(mids, line, DIST_L2, 0, 0.01, 0.01);

    double cos_theta = line[0];
    double sin_theta = line[1];
    double x0 = line[2], y0 = line[3];

    double k = sin_theta / cos_theta;
    double b = y0 - k * x0;

    double x = 0;
    double y = k * x + b;
    x0 = _image.cols;
    y0 = k * x0 + b;

    cv::line(IMG, Point(x0, y0), Point(x, y), cv::Scalar(255), 1);
    cv::imshow("img", IMG);
    cv::waitKey();
}

std::vector<std::vector<cv::Point>> CBC::FindBigestContour(cv::Mat src, int &index)
{
    int imax = -1;        //代表最大轮廓的序号
    int imaxcontour = -1; //代表最大轮廓的大小
    std::vector<std::vector<cv::Point>> contours;
    findContours(src, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
        // int itmp = contourArea(contours[i]); //这里采用的是轮廓大小
        int itmp = contours[i].size(); //这里采用的是轮廓周长
        if (imaxcontour < itmp)
        {
            imax = i;
            imaxcontour = itmp;
        }
    }
    index = imax;
    return contours;
}

double CBC::CowHeight(const cv::Mat &_image, cv::Point &_p)
{
    cv::Mat bl;
    // cv::erode(_image, bl, cv::Mat(15, 15, CV_8UC1, cv::Scalar(1)));
    cv::blur(_image, bl, cv::Size(11, 11));

    double A, B, C;
    GetLineABC(middle_line_, A, B, C);
    int xl = 0, xr = image_depth_.cols - 1;
    int yl = -(A * xl + C) / B;
    int yr = -(A * xr + C) / B;
    std::vector<cv::Point> contour1, contour2;
    int gap = 50;
    contour1.push_back(cv::Point(0, 0));
    contour1.push_back(cv::Point(image_depth_.cols - 1, 0));
    contour1.push_back(cv::Point(xr, yr - gap));
    contour1.push_back(cv::Point(xl, yl - gap));
    contour1.push_back(cv::Point(0, 0));

    contour2.push_back(cv::Point(xl, yl + gap));
    contour2.push_back(cv::Point(xr, yr + gap));
    contour2.push_back(cv::Point(image_depth_.cols - 1, image_depth_.rows - 1));
    contour2.push_back(cv::Point(0, image_depth_.rows - 1));
    contour2.push_back(cv::Point(xl, yl + gap));

    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour1);
    contours.push_back(contour2);

    double min_pixel, max_picel;
    cv::Point min_xy, max_xy;
    cv::Mat cow_roi = mask_.clone();
    cv::drawContours(cow_roi, contours, -1, cv::Scalar(0), -1);
    cv::erode(cow_roi, cow_roi, cv::Mat(15, 15, CV_8UC1, cv::Scalar(1)));

    cv::minMaxLoc(bl, &min_pixel, &max_picel, &min_xy, &_p, cow_roi);

    _p = min_xy;

    if (save_flag_ > 0)
    {
        cv::circle(IMG, _p, 11, cv::Scalar(0, 0, 0), 3);
        // cv::drawContours(IMG, contours, -1, cv::Scalar(0), 3);
        // cv::imshow("img", IMG);
        // cv::waitKey();
    }

    return camera_hight_ - min_pixel;
}

//https://www.freesion.com/article/5903825212/
cv::Point CBC::GetEquilateralTriangle(cv::Point &_U, cv::Point &_D)
{
    // 根据旋转矩阵求等边三角形第三点坐标
    cv::Point AB = _U - _D;
    double theta = 60 * CV_PI / 180;
    double R00 = cos(theta);
    double R01 = sin(theta);
    double R10 = -sin(theta);
    double R11 = cos(theta);
    return cv::Point(_D.x + R00 * AB.x + R01 * AB.y, _D.y + R10 * AB.x + R11 * AB.y);
}

void CBC::SplitUD(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, cv::Point &_U, cv::Point &_D)
{
    double A, B, C;
    GetLineABC(_line, A, B, C);

    double AcB = A / B;
    double CcB = C / B;
    std::vector<cv::Point> contour_u, contour_d;
    double max_u = 0, max_d = 0;
    for (size_t i = 0; i < _contour.size(); i++)
    {

        // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
        double dis = abs(A * _contour[i].x + B * _contour[i].y + C);
        if ((_contour[i].y + AcB * _contour[i].x + CcB) > 0)
        {
            contour_d.push_back(_contour[i]);
            if (dis > max_d)
            {
                max_d = dis;
                _D = _contour[i];
            }
        }
        else
        {
            contour_u.push_back(_contour[i]);
            if (dis > max_u)
            {
                max_u = dis;
                _U = _contour[i];
            }
        }
    }

    if (save_flag_ > 0)
    {
        // for (size_t i = 0; i < contour_d.size() - 1; i++)
        // {
        //     cv::line(IMG, contour_d[i], contour_d[i + 1], cv::Scalar(0, 0, 0), 2, 8);
        // }
        // for (size_t i = 0; i < contour_u.size() - 1; i++)
        // {
        //     cv::line(IMG, contour_u[i], contour_u[i + 1], cv::Scalar(255, 255, 255), 2, 8);
        // }
        cv::circle(IMG, _U, 11, cv::Scalar(0, 0, 0), 3);
        cv::circle(IMG, _D, 11, cv::Scalar(0, 0, 0), 3);
        // imshow("img", IMG);
        // waitKey();
    }
}

cv::Point2f CBC::GetTailPosition(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, const cv::Point &_U, const cv::Point &_D)
{
    int mid_x = (_U.x + _D.x) / 2;
    double A, B, C;
    GetLineABC(_line, A, B, C);

    cv::Point tail;
    double min_d = 1e10;
    for (size_t i = 0; i < _contour.size(); i++)
    {
        if (_contour[i].x > mid_x)
        {
            // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
            double dis = abs(A * _contour[i].x + B * _contour[i].y + C);
            if (dis < min_d)
            {
                min_d = dis;
                tail = _contour[i];
            }
        }
    }
    if (save_flag_ > 0)
    {
        cv::circle(IMG, tail, 11, cv::Scalar(0, 0, 0), 3);
        // imshow("img", IMG);
        // waitKey();
    }

    return tail;
}

void CBC::EraseRegion(const std::vector<cv::Point> &_contour, const cv::Vec4f &_line, cv::Point &_tail)
{
    double A, B, C;
    GetLineABC(_line, A, B, C);

    double AcB = A / B;
    double CcB = C / B;
    std::vector<cv::Point> contour_l, contour_r;

    double max_l = 0, max_r = 0;
    cv::Point L;
    cv::Point R;
    for (size_t i = 0; i < _contour.size(); i++)
    {

        // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
        double dis = abs(A * _contour[i].x + B * _contour[i].y + C);
        if ((_contour[i].y + AcB * _contour[i].x + CcB) > 0)
        {
            contour_r.push_back(_contour[i]);
            if (dis > max_r)
            {
                max_r = dis;
                R = _contour[i];
            }
        }
        else
        {
            contour_l.push_back(_contour[i]);
            if (dis > max_l)
            {
                max_l = dis;
                L = _contour[i];
            }
        }
    }
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour_r);
    contours.push_back(contour_l);

    cv::Mat cow_roi = Mat::zeros(image_cow_.rows, image_cow_.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, 0, cv::Scalar(1), -1);
    cv::drawContours(mask_, contours, 1, cv::Scalar(0), -1);

    // image_cow_ = image_cow_.mul(cow_roi);
    image_depth_ = image_depth_.mul(cow_roi);

    cow_roi = Mat::ones(image_cow_.rows, image_cow_.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, 0, cv::Scalar(0), -1);
    image_depth_ = image_depth_ + cow_roi;

    _tail = R;
}

void CBC::EraseRegion(const cv::Point &_U, const cv::Point &_M, const cv::Point &_D)
{
    std::vector<cv::Point> contour;
    contour.push_back(cv::Point(0, 0));
    // 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
    double y = 0;
    double x = (_M.x - _U.x) / (_M.y - _U.y) * (y - _U.y) + _U.x;
    contour.push_back(cv::Point(x, y));
    contour.push_back(_U);
    contour.push_back(_M);
    contour.push_back(_D);
    y = image_depth_.rows - 1;
    x = (_M.x - _D.x) / (_M.y - _D.y) * (y - _D.y) + _D.x;
    contour.push_back(cv::Point(x, y));
    contour.push_back(cv::Point(0, image_depth_.rows - 1));
    contour.push_back(cv::Point(0, 0));

    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);

    cv::drawContours(mask_, contours, 0, cv::Scalar(0), -1);
}
void CBC::GetLineABC(cv::Vec4f _line, double &A, double &B, double &C)
{
    A = _line[1];
    B = -_line[0];
    C = _line[0] * _line[3] - _line[1] * _line[2];
}

cv::Vec4f CBC::VerticalLine(const cv::Vec4f &_line, const cv::Point &_p)
{
    cv::Vec4f vt_line;
    float a = 1.0;
    float b = -_line[0] * a / _line[1];
    float ab = sqrt(a * a + b * b);
    a = a / ab;
    b = b / ab;
    vt_line[0] = a;
    vt_line[1] = b;
    vt_line[2] = _p.x;
    vt_line[3] = _p.y;

    return vt_line;
}

void CBC::CameraCoor()
{
#include <fstream>
    std::ofstream obj;
    obj.open("dp.obj");
    double V = 0;
    for (size_t i = 0; i < image_depth_.rows - 1; i++)
    {
        for (size_t j = 0; j < image_depth_.cols - 1; j++)
        {

            int a = image_depth_.at<uint16_t>(i, j);
            int b = image_depth_.at<uint16_t>(i, j + 1);
            int c = image_depth_.at<uint16_t>(i + 1, j);
            int d = image_depth_.at<uint16_t>(i + 1, j + 1);
            if (a > 1 && b > 1 && c > 1 && d > 1)
            {
                double Xa, Ya, Za;
                double Xb, Yb, Zb;
                double Xc, Yc, Zc;
                double Xd, Yd, Zd;
                GetCameraXYZ(j, i, a, Xa, Ya, Za);
                GetCameraXYZ(j + 1, i, b, Xb, Yb, Zb);
                GetCameraXYZ(j, i + 1, c, Xc, Yc, Zc);
                GetCameraXYZ(j + 1, i + 1, d, Xd, Yd, Zd);

                double L = abs(Xa - Xb);
                double W = abs(Ya - Yc);
                double H = (Za + Zb + Zc + Zd) / 4;
                V += (L * W * H);

                obj << "v " << Xa << " " << Ya << " " << 2 - Za << "\n";
            }
        }
    }
    std::cout << "=============\t" << V << "\n";
    obj.close();
}
void CBC::GetCameraXYZ(int _x, int _y, int _p, double &_X, double &_Y, double &_Z)
{
    _Z = _p * 0.001;
    _X = (cx_ * _Z - _Z * _x) / fx_;
    _Y = (cy_ * _Z - _Z * _y) / fy_;
}

void CBC::FillHole()
{
    int index = -1;
    cv::Mat bw = mask_.clone();
    std::vector<std::vector<cv::Point>> contours = FindBigestContour(bw, index);

    cv::fitLine(contours[index], middle_line_, DIST_L2, 0, 0.01, 0.01);

    cv::Mat cow_roi = Mat::zeros(image_cow_.rows, image_cow_.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, index, cv::Scalar(1), -1);
    if (save_flag_ > 0)
    {

        cv::drawContours(IMG, contours, index, cv::Scalar(255, 255, 255), 3);
        // imshow("img", IMG);
        // waitKey();
    }
    image_depth_ = image_depth_.mul(cow_roi);

    cow_roi = Mat::ones(image_cow_.rows, image_cow_.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, index, cv::Scalar(0), -1);
    image_depth_ = image_depth_ + (cow_roi);

    for (size_t i = 0; i < image_depth_.rows; i++)
    {
        for (size_t j = 0; j < image_depth_.cols; j++)
        {

            int d = image_depth_.at<uint16_t>(i, j);
            if (d == 0)
            {
                if (j != 0)
                {
                    image_depth_.at<uint16_t>(i, j) = image_depth_.at<uint16_t>(i, j - 1);
                }
                else
                {
                    for (size_t k = j + 1; k < image_depth_.cols; k++)
                    {
                        int t = image_depth_.at<uint16_t>(i, k);
                        if (t > 1)
                        {
                            image_depth_.at<uint16_t>(i, j) = t;
                            break;
                        }
                    }
                } //if j
                // std::cout << i << " " << j << " " << image_depth_.at<uint16_t>(i, j) << "\n";
            } //if d
        }     //for j
    }         //for i
}

int CBC::SetCamera(double _camera_hight, double _fx, double _fy, double _cx, double _cy)
{
    LOGD("SetCamera");
    camera_hight_ = _camera_hight * 1000;
    fx_ = _fx;
    fy_ = _fy;
    cx_ = _cx;
    cy_ = _cy;
    return 0;
}
int CBC::SetSaveFlag(int _flag)
{
    save_flag_ = _flag;
    return 0;
}

int CBC::ComputerHVLR(const cv::Mat &_image, double &_H, double &_VL, double &_VR)
{
    LOGD("enter ComputerHVLR");
    image_depth_ = _image;
    image_cow_ = camera_hight_ - image_depth_;
    interest_points_.clear();
    VAS_U_.clear();
    VAS_D_.clear();
    if (save_flag_ > 0)
    {
        Mat gray;
        image_cow_.convertTo(gray, CV_8UC1, 0.1);
        // gray = (gray.mul((gray < 220) / 255) - 50) * 2;
        cv::equalizeHist(gray, gray);
        applyColorMap(gray, IMG, 2);
    }

    int index = -1;
    cv::Mat bw = (image_cow_ > 1200) & (image_cow_ < 2000);
    std::vector<std::vector<cv::Point>> contours = FindBigestContour(bw, index);
    if (index == -1)
    {
        LOGD("cant find cow");
        return -1;
    }
    LOGD("contours size %d,index %d,index size %d", contours.size(), index, contours[index].size());

    mask_ = Mat::zeros(image_cow_.rows, image_cow_.cols, CV_8UC1);
    cv::drawContours(mask_, contours, index, cv::Scalar(1), -1);

    cv::Point U, D;
    cv::Vec4f line;
    cv::fitLine(contours[index], line, DIST_L2, 0, 0.01, 0.01);
    middle_line_ = line;
    SplitUD(contours[index], line, U, D);

    cv::Point mid_point = GetEquilateralTriangle(U, D);

    if (save_flag_ > 0)
    {

        cv::circle(IMG, mid_point, 11, cv::Scalar(0, 0, 0), 3);
        // imshow("img", IMG);
        // waitKey();
    }
    EraseRegion(U, mid_point, D);

    FillHole();

    cv::Point top_point;
    cow_hight_ = CowHeight(image_depth_, top_point);
    _H = cow_hight_ * 0.001;

    // CameraCoor();
    cv::Mat image_D, image_U;
    SplitImageUD(middle_line_, image_depth_, image_D, image_U);
    std::vector<double> VAS_U, VAS_D;
    VAS_U_ = ComputerVAS(image_U);
    VAS_D_ = ComputerVAS(image_D);
    _VR = VAS_U_[0];
    _VL = VAS_D_[0];

    cv::Point tail = GetTailPosition(contours[index], middle_line_, U, D);
    interest_points_.push_back(U);
    interest_points_.push_back(mid_point);
    interest_points_.push_back(D);
    interest_points_.push_back(top_point);
    interest_points_.push_back(tail);

    if (save_flag_ > 0)
    {
        drawLine(IMG, middle_line_);
        cv::imwrite("HVLR.jpg", IMG);
    }

    return 0;

    imshow("img1", IMG);
    waitKey();
    // image_depth_.convertTo(IMG, CV_8UC1, 0.1);
    imshow("img1", IMG);
    waitKey();
    std::vector<cv::Point> mids;
    mids.push_back(top_point);
    mids.push_back(mid_point);
    mids.push_back((U + D) / 2);
    // cv::fitLine(mids, line, DIST_L2, 0, 0.01, 0.01);
    line = middle_line_;
    double cos_theta = line[0];
    double sin_theta = line[1];
    double x0 = line[2], y0 = line[3];

    double k = sin_theta / cos_theta;
    double b = y0 - k * x0;

    double x = 0;
    double y = k * x + b;
    x0 = _image.cols;
    y0 = k * x0 + b;

    cv::line(IMG, Point(x0, y0), Point(x, y), cv::Scalar(255), 1);
    cv::imshow("img", IMG);
    cv::waitKey();
    return 0;
}

int CBC::ComputerAS(double &_A_U, double &_S_U, double &_A_D, double &_S_D)
{
    if (VAS_U_.size() == 3 && VAS_D_.size() == 3)
    {
        _A_U = VAS_U_[1];
        _S_U = VAS_U_[2];
        _A_D = VAS_D_[1];
        _S_D = VAS_D_[2];
    }
    else
    {
        return -1;
    }
    return 0;
}

std::vector<double> CBC::ComputerVAS(const cv::Mat &_image)
{

    double cowH = cow_hight_ * 0.001;
    double cameraH = camera_hight_ * 0.001;
    double V = 0;
    double A = 0;
    for (size_t i = 0; i < _image.rows - 1; i++)
    {
        for (size_t j = 0; j < _image.cols - 1; j++)
        {

            int a = _image.at<uint16_t>(i, j);
            int b = _image.at<uint16_t>(i, j + 1);
            int c = _image.at<uint16_t>(i + 1, j);
            int d = _image.at<uint16_t>(i + 1, j + 1);
            if (a > 1 && b > 1 && c > 1 && d > 1)
            {
                double Xa, Ya, Za;
                double Xb, Yb, Zb;
                double Xc, Yc, Zc;
                double Xd, Yd, Zd;
                GetCameraXYZ(j, i, a, Xa, Ya, Za);
                GetCameraXYZ(j + 1, i, b, Xb, Yb, Zb);
                GetCameraXYZ(j, i + 1, c, Xc, Yc, Zc);
                GetCameraXYZ(j + 1, i + 1, d, Xd, Yd, Zd);

                double L = abs(Xa - Xb);
                double W = abs(Ya - Yc);
                double H = abs(cowH - cameraH + (Za + Zb + Zc + Zd) / 4);
                V += (L * W * H);
                A += (L * W);
            }
        }
    }
    std::vector<double> VAS;
    VAS.push_back(V);
    VAS.push_back(A);
    double S = cv::sum((_image > 100) / 255)[0];
    VAS.push_back(S);
    return VAS;
}

int CBC::GetInterestPoint(cv::Point2f &_num1, cv::Point2f &_num2, cv::Point2f &_num3, cv::Point2f &_num5, cv::Point2f &_num7)
{
    if (interest_points_.size() == 5)
    {
        _num1 = interest_points_[0];
        _num2 = interest_points_[1];
        _num3 = interest_points_[2];
        _num5 = interest_points_[3];
        _num7 = interest_points_[4];
    }
    else
    {
        return -1;
    }
    return 0;
}

void CBC::SplitImageUD(const cv::Vec4f &_line, const cv::Mat &_image, cv::Mat &_image_U, cv::Mat &_image_D)
{
    double A, B, C;
    GetLineABC(_line, A, B, C);
    int xl = 0, xr = image_depth_.cols - 1;
    int yl = -(A * xl + C) / B;
    int yr = -(A * xr + C) / B;
    std::vector<cv::Point> contour;

    contour.push_back(cv::Point(0, 0));
    contour.push_back(cv::Point(image_depth_.cols - 1, 0));
    contour.push_back(cv::Point(xr, yr));
    contour.push_back(cv::Point(xl, yl));
    contour.push_back(cv::Point(0, 0));
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);

    cv::Mat cow_roi = Mat::zeros(_image.rows, _image.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, -1, cv::Scalar(1), -1);
    _image_U = _image.mul(cow_roi);

    cow_roi = Mat::ones(_image.rows, _image.cols, CV_16UC1);
    cv::drawContours(cow_roi, contours, -1, cv::Scalar(0), -1);
    _image_D = _image.mul(cow_roi);

    // _image_D.convertTo(_image_D, CV_8UC1, 0.1);
    // _image_U.convertTo(_image_U, CV_8UC1, 0.1);
    // cv::imshow("D", _image_D);
    // cv::imshow("U", _image_U);
    // cv::waitKey();
}
std::vector<cv::Point2d> midArea(const cv::Mat _image)
{
    std::vector<cv::Point2d> mids;
    for (size_t i = 0; i < _image.cols; i++)
    {
        int flag = 0;
        cv::Point2f mid_a, mid_b;
        for (size_t j = 0; j < _image.rows - 1; j++)
        {
            if (_image.at<uchar>(j, i) == 0 && _image.at<uchar>(j + 1, i) == 255)
            {
                mid_a = cv::Point2f(i, j);
                flag++;
            }

            if (_image.at<uchar>(j, i) == 255 && _image.at<uchar>(j + 1, i) == 0)
            {
                mid_b = cv::Point2f(i, j);
                flag++;
            }
        }
        if (flag == 2)
        {
            mids.push_back((mid_a + mid_b) / 2);
        }
        // else
        {
            std::cout << flag;
        }
    }
    return mids;
}

double pot2line(const cv::Point2d pt0, const cv::Point2d pt1, const cv::Point2d pt2)
{
    // 点(x0,y0)到直线Ax+By+C=0的距离为d = (A*x0+B*y0+C)/sqrt(A^2+B^2)
    double A, B, C, dis;
    // 化简两点式为一般式
    // 两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
    // 化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
    // A = y2 - y1
    // B = x1 - x2
    // C = x2y1 - x1y2
    A = pt2.y - pt1.y;
    B = pt1.x - pt2.x;
    C = pt2.x * pt1.y - pt1.x * pt2.y;
    //中心点坐标(coreX,coreY)
    double coreX, coreY;
    coreX = pt0.x;
    coreY = pt0.y;
    // 距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
    dis = abs(A * coreX + B * coreY + C) / sqrt(A * A + B * B);
    return dis;
}

void CBC::drawLine(cv::Mat &_image, const cv::Vec4f &_line)
{

    double cos_theta = _line[0];
    double sin_theta = _line[1];
    double x0 = _line[2], y0 = _line[3];

    double k = sin_theta / cos_theta;
    double b = y0 - k * x0;

    double x = 0;
    double y = k * x + b;
    x0 = _image.cols;
    y0 = k * x0 + b;

    cv::line(_image, Point(x0, y0), Point(x, y), cv::Scalar(255, 0, 0), 1); // cv::imshow("img", _image);
    // cv::waitKey();
}
