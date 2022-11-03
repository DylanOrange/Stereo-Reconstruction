#include <iostream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "Eigen.h"


using namespace std;
using namespace cv;

struct transformation{
    cv::Mat R;
    cv::Mat t;
};

enum name_set{KITTI_2011_09_26_drive_0048, KITTI_2011_09_26_drive_0113, KITTI_TEST, MATLAB_TEST, CHESSBOARD, KITTI_2015};

enum stereo_matching{SEMI_GLOBAL_MATCHING, BLOCK_MATCHING};

struct Dataset{
    int name;
    bool rectified; 
    bool distort;
    bool given_points; // directly give the match points, or detect them with keypoint detector? The former method is just for debugging
    float baseline;
    float focal_length;
};

// ORB detector
int OrbDetector (Mat img_1, Mat img_2, 
                 vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right,
                 size_t num_keypoints);

// SIFT detector
int SiftDetector (Mat img_1, Mat img_2, 
                 vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right, 
                 size_t num_keypoints);

// undistortion
void Undistort(Mat distorted_left_image, Mat distorted_right_image, 
               Mat& undistorted_left_image,  Mat& undistorted_right_image);

// Compute the essential matrix: E = K2^T * F * K1
Mat FindEssentialMatrix(Mat fundamental_mat, struct Dataset dataset);

// Choose the correct R and T from four possible solutions
struct transformation RecoverRT(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset);
struct transformation RecoverRT_2(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset, Mat& lambda);

// Get the instrinsic
void GetIntrinsics(struct Dataset dataset, cv::Mat& left_rgb_camera_matrix, cv::Mat& right_rgb_camera_matrix);

// Get the dir path of the dataset
String GetDirPath(struct Dataset dataset);

// Directly give the match points (this is just for debugging)
void GetPoints(vector<cv::Point2f>& keypoints_left, vector<cv::Point2f>& keypoints_right, struct Dataset dataset);

// Get the full path of left and right images
void getFilesList(String dirpath, vector<String> &left_image_paths, vector<String> &right_image_paths);

// Generate pointcloud
void PointCloudGenerate(cv::Mat depth_map, cv::Mat rgb_map, struct Dataset dataset, int file_order);

// rectify the images
int Rectify_KITTI(Mat R23, Mat t23, Mat left_original, Mat right_original, Mat& rectified_left, Mat& rectified_right);