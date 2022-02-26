#include "include.h"
#define DEBUG_PRINT 1

using namespace std;
using namespace cv;


// Compute essential matrix from fundamental matrix， E = K2^T * F * K1
Mat FindEssentialMatrix(Mat fundamental_mat, struct Dataset dataset){
    if(DEBUG_PRINT){
        cout<<"fundamental_mat.size(): "<<fundamental_mat.size()<<endl;
    }
    Mat left_rgb_camera_matrix, right_rgb_camera_matrix;
    GetIntrinsics(dataset, left_rgb_camera_matrix, right_rgb_camera_matrix);
    return right_rgb_camera_matrix.t() * fundamental_mat * left_rgb_camera_matrix;

}