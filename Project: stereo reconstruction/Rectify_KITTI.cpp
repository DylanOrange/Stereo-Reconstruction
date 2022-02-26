#include"include.h"

#define DEBUG_PRINT 0
#define IMAGE_SHOW 1

using namespace cv;
using namespace std;

int Rectify_KITTI(Mat R23, Mat t23, Mat left_original, Mat right_original, Mat& rectified_left, Mat& rectified_right){

    struct Dataset dataset;
    dataset.name = KITTI_TEST;
    dataset.rectified = 0;
    dataset.distort = 1;
    dataset.given_points = 0;

    Mat left_rgb_camera_matrix, right_rgb_camera_matrix;

    GetIntrinsics(dataset, left_rgb_camera_matrix, right_rgb_camera_matrix);

    // D_02
    Mat left_rgb_distortion_coeffs = (cv::Mat_<double>(5,1) << -3.691481e-01, 1.968681e-01, 1.353473e-03, 5.677587e-04, -6.770705e-02);

    // D_03
    Mat right_rgb_distortion_coeffs = (cv::Mat_<double>(5,1) << -3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02);


    
    cv::Mat R1, R2, P1, P2, Q;

    cv::stereoRectify(left_rgb_camera_matrix, left_rgb_distortion_coeffs, right_rgb_camera_matrix, right_rgb_distortion_coeffs,
                      left_original.size(), R23, t23, R1, R2, P1, P2, Q);
    
    // LOG(INFO) << "ROTATION FOR THE FIRST CAMERA :" << R1;
    // LOG(INFO) << "ROTATION FOR THE SECOND CAMERA :" << R2;
    // LOG(INFO) << "R1 IMAGE TYPE: " << R1.type();

    // auto extrin_ir2rectified_ir = Eigen::Isometry3f::Identity();
    // for (int i = 0; i < 3; i ++) {
    //     for (int j = 0; j < 3; j++) {
    //         extrin_ir2rectified_ir.matrix()(i,j) = float(R1.at<double>(i,j));
    //     }
    // }

    // LOG(INFO) << "IR 2 RECTIFIED_IR TRANSFORMATION MATRIX " << extrin_ir2rectified_ir.matrix();

    cv::Mat rmap[2][2];
    cv::initUndistortRectifyMap(left_rgb_camera_matrix, left_rgb_distortion_coeffs, R1, P1, left_original.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(right_rgb_camera_matrix, right_rgb_distortion_coeffs, R2, P2, left_original.size(), CV_16SC2, rmap[1][0], rmap[1][1]);

    // cv::Mat rectified_left;
    // cv::Mat rectified_right;
    cv::remap(left_original, rectified_left, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
    cv::remap(right_original, rectified_right, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

    cv::imshow("left_rectified", rectified_left);
    cv::imshow("right_rectified", rectified_right);
    cv::waitKey();

    // cv::imwrite("/home/xyz/left_rectified.png", rectified_left);
    // cv::imwrite("/home/xyz/right_rectified.png", rectified_right);
    
    
    return 0; 
}
