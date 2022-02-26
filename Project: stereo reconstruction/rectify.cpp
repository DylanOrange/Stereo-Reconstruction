#include"include.h"

#define DEBUG_PRINT 0
#define IMAGE_SHOW 1

using namespace cv;
using namespace std;

int main(int argc, char*argv[]){
    String left = "./test_images/kitti_test_01/image_02/data/left_unrectified.png";
    String right = "./test_images/kitti_test_01/image_03/data/right_unrectified.png";
    Mat left_original = imread ( left, CV_LOAD_IMAGE_COLOR );
    Mat right_original = imread ( right, CV_LOAD_IMAGE_COLOR );
    Mat rectified_left, rectified_right;
    Mat R23 = (cv::Mat_<double>(3,3) <<  0.999371,   0.0210911,  -0.0285063,
                                       -0.0210506,   0.999777,    0.00172109,
                                        0.0285362,  -0.00111993,  0.999592);

    Mat t23 = (cv::Mat_<double>(3,1) <<  -0.523653, 0.010085, -0.0973426);
    Rectify_KITTI(R23, t23, left_original, right_original, rectified_left, rectified_right);

    return 0; 
}