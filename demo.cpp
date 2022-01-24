#include "cbc.hpp"
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

    cv::Mat image = imread(argv[1], IMREAD_UNCHANGED);
    CBC cbc;
    cbc.SetDepth(image);
    // getchar();

    return 0;
}
