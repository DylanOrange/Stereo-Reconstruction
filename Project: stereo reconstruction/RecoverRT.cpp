#include"include.h"
#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace cv;
using namespace std;

bool JudgeRT(struct transformation transformation, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, double& scale, struct Dataset dataset);

struct transformation RecoverRT(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset){
    assert(keypoints_left.size() == keypoints_right.size());

    

    // Mat T_ = T.resize(3);
    // T = T.t();
    if(DEBUG_PRINT){
        cout<<"RecoverRT"<<endl;
        cout<<"R1: "<<R1<<endl;
        cout<<"R2: "<<R2<<endl;
        cout<<"T: "<<T<<endl;
        cout<<"T.size(): "<<T.size()<<endl;
        cout<<"T.type(): "<<T.type()<<endl;  // CV_64F
    }
    struct transformation transformation_1, transformation_2, transformation_3, transformation_4, transformation_return;
    std::vector<struct transformation> transformations;
    transformation_1.R = R1; transformation_1.t = T;  transformations.push_back(transformation_1);
    transformation_2.R = R1; transformation_2.t = -T; transformations.push_back(transformation_2);
    transformation_3.R = R2; transformation_3.t = T;  transformations.push_back(transformation_3);
    transformation_4.R = R2; transformation_4.t = -T; transformations.push_back(transformation_4);
    int count_success = 0;
    double scale = 0.0;
    for(auto transformation:transformations){
        if(JudgeRT(transformation, keypoints_left, keypoints_right, scale, dataset)){
            count_success++;
            // transformation.t *= scale;
            double baseline = 0.5327190420453419;  // baseline of KITTI
            transformation.t *= baseline / norm(transformation.t);
            transformation_return = transformation;
        }
    }
    if(DEBUG_PRINT){
        cout<<"count_success: "<<count_success<<endl;
        cout<<"R: "<<transformation_return.R<<endl;
        cout<<"t: "<<transformation_return.t<<endl;
        cout<<"scale: "<<scale<<endl;
    }
    // assert(count_success == 1);
    if(count_success != 1){
        cout<<"the number of the correct solution of R and t should be 1"<<endl;
        // exit(0);
    }
    return transformation_return;
}

bool JudgeRT(struct transformation transformation, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, 
             double& scale, struct Dataset dataset){

    Mat left_rgb_camera_matrix, right_rgb_camera_matrix;
    GetIntrinsics(dataset, left_rgb_camera_matrix, right_rgb_camera_matrix); 
    cv::Mat left_rgb_camera_matrix_inv = left_rgb_camera_matrix.inv();
    cv::Mat right_rgb_camera_matrix_inv = right_rgb_camera_matrix.inv();

    // 构建M矩阵
    size_t num_points = keypoints_left.size();
    Mat M = Mat::zeros(cv::Size(num_points+1, 3*num_points), CV_64FC1);
    Mat _x1 = Mat::zeros(cv::Size(1,3), CV_64FC1);
    Mat x1;
    Mat _x2 = Mat::zeros(cv::Size(1,3), CV_64FC1);
    Mat x2;
    Mat x2_hat = Mat::zeros(cv::Size(3,3), CV_64FC1);
    Mat tmp1; // x2_hat * R * x1 
    Mat tmp2; // x2_hat * T
    Mat R = transformation.R;
    Mat t = transformation.t;
    for(int i = 0; i < num_points; i++){
        // Mat tmp1 = Mat::zeros(cv::Size(3,1), CV_64FC1); 
        _x1.at<double>(0,0) = keypoints_left[i].x;
        _x1.at<double>(1,0) = keypoints_left[i].y;
        _x1.at<double>(2,0) = 1.0;
        x1 = left_rgb_camera_matrix_inv * _x1;

        _x2.at<double>(0,0) = keypoints_right[i].x;
        _x2.at<double>(1,0) = keypoints_right[i].y;
        _x2.at<double>(2,0) = 1.0;
        x2 = right_rgb_camera_matrix_inv * _x2;

        x2_hat.at<double>(0,1) = -1.0;
        x2_hat.at<double>(0,2) = x2.at<double>(1,0);
        x2_hat.at<double>(1,0) = 1.0;
        x2_hat.at<double>(1,2) = -x2.at<double>(0,0);
        x2_hat.at<double>(2,0) = -x2.at<double>(1,0);
        x2_hat.at<double>(2,1) = x2.at<double>(0,0);

        if(DEBUG_PRINT){
            // cout<<"x2_hat.size(): "<<x2_hat.size()<<endl;
            // cout<<"x2_hat.type(): "<<x2_hat.type()<<endl;
            // cout<<"R.size(): "<<R.size()<<endl;
            // cout<<"x1.size(): "<<x1.size()<<endl;
            // cout<<"x1.type(): "<<x1.type()<<endl;
            // cout<<"t.size(): "<<t.size()<<endl;
            // cout<<"x1: "<<x1<<endl;
            // cout<<"x2: "<<x2<<endl;
        }

        tmp1 = x2_hat * R * x1;
        tmp2 = x2_hat * t;

        if(DEBUG_PRINT){
            // cout<<"tmp1.type(): "<<tmp1.type()<<endl; // CV_64FC1
            // cout<<"tmp2.type(): "<<tmp2.type()<<endl; // CV_64FC1
        }

        M.at<double>(3*i, i) =  tmp1.at<double>(0,0);
        M.at<double>(3*i+1, i) = tmp1.at<double>(1,0);    
        M.at<double>(3*i+2, i) = tmp1.at<double>(2,0);
        M.at<double>(3*i, num_points) =  tmp2.at<double>(0,0);
        M.at<double>(3*i+1, num_points) = tmp2.at<double>(1,0);    
        M.at<double>(3*i+2, num_points) = tmp2.at<double>(2,0);   
        
    }

    // 由 M^T * M 的特征值分解得到 M * lambda = 0 的解
    Mat M_M = M.t() * M;
    Mat E, V;
    cv::eigen(M_M, E, V);
    if(DEBUG_PRINT){
        // cout<<"E.size(): "<<E.size()<<endl; // eigenvalue
        // // cout<<"Eigenvalues: "<<E<<endl;
        // cout<<"V.size(): "<<V.size()<<endl; // eigenvector
        // cout<<"V.type(): "<<V.type()<<endl; 
    }
    // Mat lambda = V.row(num_points);
    Mat lambda = Mat::zeros(cv::Size(1,num_points), CV_64FC1);
    for(int i = 0; i<num_points; i++){
        lambda.at<double>(i,0) = V.at<double>(num_points,i);
    }
    double gamma = V.at<double>(num_points, num_points);  // scale factor
    // double gamma = (V.at<double>(num_points-2, num_points) + V.at<double>(num_points-1, num_points) + V.at<double>(num_points, num_points))/3;

    // 根据符号来判断该R和T是否正确
    if(gamma < 0){
        gamma = -gamma;
        lambda = -lambda;
    }
    // lambda = lambda / gamma;
    if(DEBUG_PRINT){
        // cout<<"lambda.type(): "<<lambda.type()<<endl;
    }

    int success_1 = 0;
    int success_2 = 0;
    for(int i = 0; i < num_points; i++){
        if(lambda.at<double>(i,0) >= 0.0){
            success_1 ++;
        }
        _x1.at<double>(0,0) = keypoints_left[i].x;
        _x1.at<double>(1,0) = keypoints_left[i].y;
        _x1.at<double>(2,0) = 1.0;
        x1 = left_rgb_camera_matrix_inv * _x1;
        x1 = x1 * lambda.at<double>(i,0);
        x2 = R * x1 + t * gamma;
        if(x2.at<double>(2,0) >= 0){
            success_2 ++;
        }
    }
    if(success_1 == num_points && success_2 == num_points){
        scale = gamma;
        return 1;
    }

    return 0;
}

bool JudgeRT_2(struct transformation transformation, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, 
             double& scale, struct Dataset dataset, Mat& lambda){

    Mat left_rgb_camera_matrix, right_rgb_camera_matrix;
    GetIntrinsics(dataset, left_rgb_camera_matrix, right_rgb_camera_matrix); 
    cv::Mat left_rgb_camera_matrix_inv = left_rgb_camera_matrix.inv();
    cv::Mat right_rgb_camera_matrix_inv = right_rgb_camera_matrix.inv();

    // 构建M矩阵
    size_t num_points = keypoints_left.size();
    Mat M = Mat::zeros(cv::Size(num_points+1, 3*num_points), CV_64FC1);
    Mat _x1 = Mat::zeros(cv::Size(1,3), CV_64FC1);
    Mat x1;
    Mat _x2 = Mat::zeros(cv::Size(1,3), CV_64FC1);
    Mat x2;
    Mat x2_hat = Mat::zeros(cv::Size(3,3), CV_64FC1);
    Mat tmp1; // x2_hat * R * x1 
    Mat tmp2; // x2_hat * T
    Mat R = transformation.R;
    Mat t = transformation.t;
    for(int i = 0; i < num_points; i++){
        // Mat tmp1 = Mat::zeros(cv::Size(3,1), CV_64FC1); 
        _x1.at<double>(0,0) = keypoints_left[i].x;
        _x1.at<double>(1,0) = keypoints_left[i].y;
        _x1.at<double>(2,0) = 1.0;
        x1 = left_rgb_camera_matrix_inv * _x1;

        _x2.at<double>(0,0) = keypoints_right[i].x;
        _x2.at<double>(1,0) = keypoints_right[i].y;
        _x2.at<double>(2,0) = 1.0;
        x2 = right_rgb_camera_matrix_inv * _x2;

        x2_hat.at<double>(0,1) = -1.0;
        x2_hat.at<double>(0,2) = x2.at<double>(1,0);
        x2_hat.at<double>(1,0) = 1.0;
        x2_hat.at<double>(1,2) = -x2.at<double>(0,0);
        x2_hat.at<double>(2,0) = -x2.at<double>(1,0);
        x2_hat.at<double>(2,1) = x2.at<double>(0,0);

        if(DEBUG_PRINT){
            // cout<<"x2_hat.size(): "<<x2_hat.size()<<endl;
            // cout<<"x2_hat.type(): "<<x2_hat.type()<<endl;
            // cout<<"R.size(): "<<R.size()<<endl;
            // cout<<"x1.size(): "<<x1.size()<<endl;
            // cout<<"x1.type(): "<<x1.type()<<endl;
            // cout<<"t.size(): "<<t.size()<<endl;
            // cout<<"x1: "<<x1<<endl;
            // cout<<"x2: "<<x2<<endl;
        }

        tmp1 = x2_hat * R * x1;
        tmp2 = x2_hat * t;

        if(DEBUG_PRINT){
            // cout<<"tmp1.type(): "<<tmp1.type()<<endl; // CV_64FC1
            // cout<<"tmp2.type(): "<<tmp2.type()<<endl; // CV_64FC1
        }

        M.at<double>(3*i, i) =  tmp1.at<double>(0,0);
        M.at<double>(3*i+1, i) = tmp1.at<double>(1,0);    
        M.at<double>(3*i+2, i) = tmp1.at<double>(2,0);
        M.at<double>(3*i, num_points) =  tmp2.at<double>(0,0);
        M.at<double>(3*i+1, num_points) = tmp2.at<double>(1,0);    
        M.at<double>(3*i+2, num_points) = tmp2.at<double>(2,0);   
        
    }

    // 由 M^T * M 的特征值分解得到 M * lambda = 0 的解
    Mat M_M = M.t() * M;
    Mat E, V;
    cv::eigen(M_M, E, V);
    if(DEBUG_PRINT){
        // cout<<"E.size(): "<<E.size()<<endl; // eigenvalue
        // // cout<<"Eigenvalues: "<<E<<endl;
        // cout<<"V.size(): "<<V.size()<<endl; // eigenvector
        // cout<<"V.type(): "<<V.type()<<endl; 
    }
    // Mat lambda = V.row(num_points);
    // Mat lambda = Mat::zeros(cv::Size(1,num_points), CV_64FC1);
    for(int i = 0; i<num_points; i++){
        lambda.at<double>(i,0) = V.at<double>(num_points,i);
    }
    double gamma = V.at<double>(num_points, num_points);  // scale factor
    // double gamma = (V.at<double>(num_points-2, num_points) + V.at<double>(num_points-1, num_points) + V.at<double>(num_points, num_points))/3;

    // 根据符号来判断该R和T是否正确
    if(gamma < 0){
        gamma = -gamma;
        lambda = -lambda;
    }
    // lambda = lambda / gamma;
    if(DEBUG_PRINT){
        // cout<<"lambda.type(): "<<lambda.type()<<endl;
    }

    int success_1 = 0;
    int success_2 = 0;
    for(int i = 0; i < num_points; i++){
        if(lambda.at<double>(i,0) >= 0.0){
            success_1 ++;
        }
        _x1.at<double>(0,0) = keypoints_left[i].x;
        _x1.at<double>(1,0) = keypoints_left[i].y;
        _x1.at<double>(2,0) = 1.0;
        x1 = left_rgb_camera_matrix_inv * _x1;
        x1 = x1 * lambda.at<double>(i,0);
        x2 = R * x1 + t * gamma;
        if(x2.at<double>(2,0) >= 0){
            success_2 ++;
        }
    }
    if(success_1 == num_points && success_2 == num_points){
        scale = gamma;
        return 1;
    }

    return 0;
}

struct transformation RecoverRT_2(Mat R1, Mat R2, Mat T, vector<Point2f> keypoints_left, vector<Point2f> keypoints_right, struct Dataset dataset, Mat& lambda){
    assert(keypoints_left.size() == keypoints_right.size());

    // Mat T_ = T.resize(3);
    // T = T.t();
    if(DEBUG_PRINT){
        cout<<"RecoverRT"<<endl;
        cout<<"R1: "<<R1<<endl;
        cout<<"R2: "<<R2<<endl;
        cout<<"T: "<<T<<endl;
        cout<<"T.size(): "<<T.size()<<endl;
        cout<<"T.type(): "<<T.type()<<endl;  // CV_64F
    }
    struct transformation transformation_1, transformation_2, transformation_3, transformation_4, transformation_return;
    std::vector<struct transformation> transformations;
    transformation_1.R = R1; transformation_1.t = T;  transformations.push_back(transformation_1);
    transformation_2.R = R1; transformation_2.t = -T; transformations.push_back(transformation_2);
    transformation_3.R = R2; transformation_3.t = T;  transformations.push_back(transformation_3);
    transformation_4.R = R2; transformation_4.t = -T; transformations.push_back(transformation_4);
    int count_success = 0;
    double scale = 0.0;
    for(auto transformation:transformations){
        if(JudgeRT_2(transformation, keypoints_left, keypoints_right, scale, dataset, lambda)){
            count_success++;
            // transformation.t *= scale;
            double baseline = 0.5327190420453419;  // baseline of KITTI
            transformation.t *= baseline / norm(transformation.t);
            transformation_return = transformation;
            lambda *= (baseline/scale);
        }
    }
    if(DEBUG_PRINT){
        cout<<"count_success: "<<count_success<<endl;
        cout<<"R: "<<transformation_return.R<<endl;
        cout<<"t: "<<transformation_return.t<<endl;
        cout<<"scale: "<<scale<<endl;
    }
    // assert(count_success == 1);
    if(count_success != 1){
        cout<<"the number of the correct solution of R and t should be 1"<<endl;
        // exit(0);
    }
    return transformation_return;
}

