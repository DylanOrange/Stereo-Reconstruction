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
    bool distort; // 是否畸变
    bool given_points; // 是否手动给特征点
    float baseline;
    float focal_length;
};

/**
 * 使用ORB进行特征点检测
 * @param img_1 左图
 * @param img_2 右图
 * @param keypoints_left 左特征点
 * @param num_keypoints 选择特征点的数量（最相似的num_keypoints个）
 */
int OrbDetector (Mat img_1, Mat img_2, 
                 vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right,
                 size_t num_keypoints);


int SiftDetector (Mat img_1, Mat img_2, 
                 vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right, 
                 size_t num_keypoints);

/**
 * 对图像进行去畸变
 */
void Undistort(Mat distorted_left_image, Mat distorted_right_image, 
               Mat& undistorted_left_image,  Mat& undistorted_right_image);

// 由基础矩阵计算本质矩阵， E = K2^T * F * K1
Mat FindEssentialMatrix(Mat fundamental_mat, struct Dataset dataset);

// 从R和t的4种可能组合中获取正确的解(根据CV2的slide 6中的方法)
struct transformation RecoverRT(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset);
struct transformation RecoverRT_2(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset, Mat& lambda);

// 获取相机内参
void GetIntrinsics(struct Dataset dataset, cv::Mat& left_rgb_camera_matrix, cv::Mat& right_rgb_camera_matrix);

// 获取数据集的路径
String GetDirPath(struct Dataset dataset);

// 手动输入特征点坐标
void GetPoints(vector<cv::Point2f>& keypoints_left, vector<cv::Point2f>& keypoints_right, struct Dataset dataset);

// 获取数据集中图片的完整路径
void getFilesList(String dirpath, vector<String> &left_image_paths, vector<String> &right_image_paths);


void PointCloudGenerate(cv::Mat depth_map, cv::Mat rgb_map, struct Dataset dataset, int file_order);

int Rectify_KITTI(Mat R23, Mat t23, Mat left_original, Mat right_original, Mat& rectified_left, Mat& rectified_right);