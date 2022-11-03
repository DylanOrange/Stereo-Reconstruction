#include "include.h"

#define DEBUG_PRINT 1
#define IMAGE_SHOW 1

using namespace std;
using namespace cv;

// K_02
Mat left_rgb_camera_matrix = (cv::Mat_<float>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);

// K_03
Mat right_rgb_camera_matrix = (cv::Mat_<float>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);

float t23[3] = {-0.5327119287764802, 0.002752896950590371, -1.597899999998976e-05};

cv::Mat left_rgb_camera_matrix_inv = left_rgb_camera_matrix.inv();

int main(int argc, char*argv[]){

    // [100-1200, 50-450, 1-40]
    float LO_U, HI_U, LO_V, HI_V, LO_Z, HI_Z;
    LO_U = 100.0;
    HI_U = 1200.0; 
    LO_V = 50.0;
    HI_V = 450.0; 
    LO_Z = 1.0;
    HI_Z = 40.0;
    // float r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
    int num_points = 440; 
    float u1[num_points], v1[num_points], z1[num_points];
    vector<Point3f> points;
    Point3f p;
    Mat _p = Mat::zeros(cv::Size(1,3), CV_32FC1);
    Mat p_;
    Mat p_2 = Mat::zeros(cv::Size(1,3), CV_32FC1);

    vector<Point2f> keypoints_left; 
    vector<Point2f> keypoints_right;
    Point2f keypoint_left, keypoint_right;

    for(int i=0; i<num_points; i++){
        u1[i] = LO_U + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_U-LO_U)));
        v1[i] = LO_V + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_V-LO_V)));
        z1[i] = LO_Z + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI_Z-LO_Z)));
        // cout<<"u1: "<<u1[i]<<" v1: "<<v1[i]<<" z1: "<<z1[i]<<endl;
        _p.at<float>(0,0) = u1[i];
        _p.at<float>(1,0) = v1[i];
        _p.at<float>(2,0) = 1.0;



        p_ = left_rgb_camera_matrix_inv * _p;

        p_2.at<float>(0,0) = p_.at<float>(0,0) * z1[i] + t23[0];
        p_2.at<float>(1,0) = p_.at<float>(1,0) * z1[i] + t23[1];
        p_2.at<float>(2,0) = z1[i] + t23[2];

        p_2 = right_rgb_camera_matrix * p_2;
        p_2.at<float>(0,0) /= p_2.at<float>(2,0);
        p_2.at<float>(1,0) /= p_2.at<float>(2,0);
        p_2.at<float>(2,0) = 1.0;

        // cout<<"u1_: "<<p_2.at<float>(0,0)<<" v1_: "<<p_2.at<float>(1,0)<<" z1_: "<<p_2.at<float>(2,0)<<endl;
        if(p_2.at<float>(0,0)<0 || p_2.at<float>(0,0)>1392 || p_2.at<float>(1,0) <0 || p_2.at<float>(1,0)>512){
            cout<<"### error ### "<<endl;
            // exit(0);
        }
        else{
            keypoint_left.x = _p.at<float>(0,0);
            keypoint_left.y = _p.at<float>(1,0);
            keypoints_left.push_back(keypoint_left);

            keypoint_right.x = p_2.at<float>(0,0);
            keypoint_right.y = p_2.at<float>(1,0);
            keypoints_right.push_back(keypoint_right);
        }

    }

    struct Dataset dataset;
    // dataset.name = KITTI_TEST; 
    dataset.name = KITTI_TEST;
    dataset.rectified = 1;
    dataset.distort = 0;
    dataset.given_points = 1; 


    Mat fundamental_mat = findFundamentalMat(keypoints_left, keypoints_right,  cv::FM_8POINT);
    // cout<<"fundamental_mat: " << fundamental_mat << endl;
    Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);
    // cout<<"essential_mat: "<<essential_mat<<endl;

    Mat R1, R2, T;
    decomposeEssentialMat(essential_mat, R1, R2, T);

    struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left, keypoints_right, dataset);
    if(DEBUG_PRINT){
        cout<<"R and t from eight-point method:"<<endl;
        cout<<"R: "<<transformation.R<<endl;
        cout<<"t: "<<transformation.t<<endl<<endl;
    }


    return 0; 
}