#include"include.h"
#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace cv;
using namespace std;

void Undistort(Mat distorted_left_image, Mat distorted_right_image, 
               Mat& undistorted_left_image,  Mat& undistorted_right_image){
                   

    //KITTI Parameters
    // K_02
    cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.597910e+02, 0.000000e+00, 6.960217e+02, 
                                                                0.000000e+00, 9.569251e+02, 2.241806e+02, 
                                                                0.000000e+00, 0.000000e+00, 1.000000e+00);

    // K_03
    cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 
                                                                0.000000e+00, 9.019653e+02, 2.242509e+02, 
                                                                0.000000e+00, 0.000000e+00, 1.000000e+00);

    // D_02
    cv::Mat left_rgb_distortion_coeffs = (cv::Mat_<double>(5,1) << -3.691481e-01, 1.968681e-01, 1.353473e-03, 5.677587e-04, -6.770705e-02);

    // D_03
    cv::Mat right_rgb_distortion_coeffs = (cv::Mat_<double>(5,1) << -3.639558e-01, 1.788651e-01, 6.029694e-04, -3.922424e-04, -5.382460e-02);


    Mat new_left_camera_matrix, new_right_camera_matrix;
    cv::undistort(distorted_left_image, undistorted_left_image, left_rgb_camera_matrix, left_rgb_distortion_coeffs, left_rgb_camera_matrix);
    cv::undistort(distorted_right_image, undistorted_right_image, right_rgb_camera_matrix, right_rgb_distortion_coeffs, right_rgb_camera_matrix);
    
    if(DEBUG_PRINT){
        cout<<"new_left_camera_matrix: "<<new_left_camera_matrix<<endl;
        cout<<"new_right_camera_matrix: "<<new_right_camera_matrix<<endl;
    }
    if(IMAGE_SHOW){
        // imshow("distorted_left_image",distorted_left_image);
        imshow("undistorted_left_image",undistorted_left_image);
        imshow("undistorted_right_image",undistorted_right_image);
        imwrite("undistorted_left_image.png", undistorted_left_image);
        imwrite("undistorted_right_image.png", undistorted_right_image);
        waitKey(0);
    }

}


