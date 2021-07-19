#include "include.h"

using namespace std;
using namespace cv;

// 参考： http://ros-developer.com/2019/01/01/decomposing-projection-using-opencv-and-c/
// 验证了rectify的相机参数

// 这个程序主要用于得到rectified后的KITTI图像的相机内参, 由结果可以确信rectified后两个相机的orientation相同， translation为0.5327190420453419米，这个结果和论文中的标注接近

int main(int argc, char*argv[]){
    Mat P_2 = (cv::Mat_<double>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
                                        0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
                                        0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03);
    Mat K_2, R_2, _t_2;
    decomposeProjectionMatrix(P_2, K_2, R_2, _t_2);
    // cout<<"_t_2.type(): "<<_t_2.type()<<endl; // double
    Mat t_2 = Mat::zeros(Size(1,3), CV_64FC1);
    for(int i = 0; i<3; i++){
        t_2.at<double>(i,0) = _t_2.at<double>(i,0)/_t_2.at<double>(3,0);
    }
    cout<<"rectified left K: "<<K_2<<endl;
    cout<<"\nR: "<<R_2<<endl;
    cout<<"\nt: "<<t_2<<endl;

    Mat P_3 = (cv::Mat_<double>(3,4) << 7.215377e+02, 0.000000e+00, 6.095593e+02, -3.395242e+02,
                                        0.000000e+00, 7.215377e+02, 1.728540e+02, 2.199936e+00,
                                        0.000000e+00, 0.000000e+00, 1.000000e+00, 2.729905e-03);
    Mat K_3, R_3, _t_3;
    decomposeProjectionMatrix(P_3, K_3, R_3, _t_3);
    Mat t_3 = Mat::zeros(Size(1,3), CV_64FC1);
    for(int i = 0; i<3; i++){
        t_3.at<double>(i,0) = _t_3.at<double>(i,0)/_t_3.at<double>(3,0);
    }
    cout<<"rectified right K: "<<K_3<<endl;
    cout<<"\nR: "<<R_3<<endl;
    cout<<"\nt: "<<t_3<<endl;
    cout<<"\nt_23: "<<t_2-t_3<<endl;
    return 0;
}


// // output log
// rectified left K: [721.5377, 0, 609.5593;
//  0, 721.5377, 172.854;
//  0, 0, 1]

// R: [1, 0, 0;
//  0, 1, 0;
//  0, 0, 1]

// t: [-0.05984926480082582;
//  0.000357927150495392;
//  -0.002745884000000004]


// rectified right K: [721.5377, 0, 609.5593;
//  0, 721.5377, 172.854;
//  0, 0, 1]

// R: [1, 0, 0;
//  0, 1, 0;
//  0, 0, 1]

// t: [0.4728626639756544;
//  -0.002394969800094979;
//  -0.002729905000000014]

// t_23: [-0.5327119287764802;
//  0.002752896950590371;
//  -1.597899999998976e-05]

// -0.5327119287764802, 0.002752896950590371, -1.597899999998976e-05
// norm: 0.5327190420453419 -> baseline
