#include"include.h"

using namespace std;
using namespace cv;

void GetIntrinsics(struct Dataset dataset, cv::Mat& left_rgb_camera_matrix, cv::Mat& right_rgb_camera_matrix){
    if(dataset.name == KITTI_2011_09_26_drive_0048 || dataset.name == KITTI_2011_09_26_drive_0113 || 
       dataset.name == KITTI_TEST || dataset.name == CHESSBOARD || dataset.name == KITTI_2015){
        if(dataset.rectified == 1){
            // rectified camera parameters
            // K_02
            left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);

            // K_03
            right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);
        }
        else{
            // K_02
            left_rgb_camera_matrix = (cv::Mat_<double>(3,3) <<  9.597910e+02, 0.000000e+00, 6.960217e+02, 
                                                                0.000000e+00, 9.569251e+02, 2.241806e+02, 
                                                                0.000000e+00, 0.000000e+00, 1.000000e+00);

            // K_03
            right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 
                                                                0.000000e+00, 9.019653e+02, 2.242509e+02, 
                                                                0.000000e+00, 0.000000e+00, 1.000000e+00);
        }
    }
    else if(dataset.name == MATLAB_TEST){
        // matlab
        // K_02
        left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 844.310547, 0, 243.413315, 0, 1202.508301, 281.529236, 0, 0, 1);

        // K_03
        right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 852.721008, 0, 252.021805, 0, 1215.657349, 288.587189, 0, 0, 1);
    }
}


String GetDirPath(struct Dataset dataset){
    String dir_path;
    if(dataset.name == KITTI_2011_09_26_drive_0048){
        if(dataset.rectified == 1){
            dir_path = "/home/dekai/datasets/2011_09_26_drive_0048/2011_09_26_drive_0048_sync/2011_09_26/2011_09_26_drive_0048_sync";
        }
        else{
            dir_path = "/home/dekai/datasets/2011_09_26_drive_0048/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract";
        }
    }
    else if(dataset.name == KITTI_2011_09_26_drive_0113){
        if(dataset.rectified == 1){
            dir_path = "/home/dekai/datasets/2011_09_26_drive_0113/2011_09_26_drive_0113_sync/2011_09_26/2011_09_26_drive_0113_sync";
        }
        else{
            dir_path = "/home/dekai/datasets/2011_09_26_drive_0113/2011_09_26_drive_0113_extract/2011_09_26/2011_09_26_drive_0113_extract";
        }
    }
    else if(dataset.name == KITTI_TEST){
        if(dataset.rectified == 1){
            dir_path = "../test_images/kitti_test_01";
        }
        else{
            dir_path = "../test_images/kitti_test_01";
        }
    }
    else if(dataset.name == MATLAB_TEST){
        dir_path = "../test_images/matlab";
    }
    
    else if(dataset.name == CHESSBOARD){
        dir_path = "../test_images/chessboard";
    }
    else if(dataset.name == KITTI_2015){
        if(dataset.rectified == 1){
            dir_path = "../test_images/kitti_2015/rectified";
        }
        else{
            dir_path = "../test_images/kitti_2015/unrectified";
        }
    }

    if(dir_path.length() == 0){
        cout<<"the dataset doesn't exist!"<<endl;
        exit(0);
    }
    return dir_path;
}

// 手动给定特征点坐标
void GetPoints(vector<cv::Point2f>& keypoints_left, vector<cv::Point2f>& keypoints_right, struct Dataset dataset){
    float x1[12], y1[12], x2[12], y2[12];
    if(dataset.name == MATLAB_TEST){
        // matlab
        float _x1[12] = {10.0000, 92.0000, 8.0000, 92.0000, 289.0000, 354.0000, 289.0000, 353.0000, 69.0000, 294.0000, 44.0000, 336.0000};
        float _y1[12] = {232.0000, 230.0000, 334.0000, 333.0000, 230.0000, 278.0000, 340.0000, 332.0000, 90.0000, 149.0000, 475.0000, 433.0000};
        float _x2[12] = {123.0000, 203.0000, 123.0000, 202.0000, 397.0000, 472.0000, 398.0000, 472.0000, 182.0000, 401.0000, 148.0000, 447.0000};
        float _y2[12] = {239.0000, 237.0000, 338.0000, 338.0000, 236.0000, 286.0000, 348.0000, 341.0000,  99.0000, 153.0000, 471.0000, 445.0000};
        for(int i=0; i<12; i++){
            x1[i] = _x1[i]; y1[i] = _y1[i]; x2[i] = _x2[i]; y2[i] = _y2[i];
        }
    }
    else if(dataset.name == KITTI_TEST){
        // rectified KITTI - 2011_09_26_drive_0113_sync - 0.png
        // float _x1[12] = {290.0, 385.0, 390.0, 426.0, 440.0, 428.0, 444.0, 962.0, 973.0, 216.0, 696.0, 132.0};
        // float _y1[12] = {111.0, 57.0, 186.0, 69.0, 98.0, 136.0, 165.0, 241.0, 55.0, 312.0, 207.0, 236.0};
        // float _x2[12] = {259.0, 370.0, 373.0, 410.0, 428.0, 413.0, 429.0, 893.0, 949.0, 126.0, 675.0, 114.0};
        // float _y2[12] = {111.0, 57.0, 187.0, 69.0, 98.0, 137.0, 165.0, 241.0, 57.0, 331.0, 209.0, 236.0};
        float _x1[12] = {290.0, 385.0, 390.0, 426.0, 440.0, 428.0, 444.0, 962.0, 973.0, 497.0, 696.0, 132.0};
        float _y1[12] = {111.0, 57.0, 186.0, 69.0, 98.0, 136.0, 165.0, 241.0, 55.0, 276.0, 207.0, 236.0};
        float _x2[12] = {259.0, 370.0, 373.0, 410.0, 428.0, 413.0, 429.0, 893.0, 949.0, 457.0, 675.0, 114.0};
        float _y2[12] = {111.0, 57.0, 187.0, 69.0, 98.0, 137.0, 165.0, 241.0, 57.0, 277.0, 209.0, 236.0};
        for(int i=0; i<12; i++){
            x1[i] = _x1[i]; y1[i] = _y1[i]; x2[i] = _x2[i]; y2[i] = _y2[i];
        }
    }
    else if(dataset.name == CHESSBOARD){
        // unrectified KITTI - chessboard
        float _x1[12] = {227.0, 1225.0, 1190.0, 1082.0, 956.0, 646.0, 536.0, 450.0, 583.0, 820.0, 830.0, 881.0};
        float _y1[12] = {199.0, 282.0, 92.0, 435.0, 472.0, 465.0, 466.0, 290.0, 106.0, 151.0, 323.0, 403.0};
        float _x2[12] = {131.0, 1101.0, 1037.0, 947.0, 827.0, 544.0, 439.0, 378.0, 481.0, 727.0, 742.0, 791.0};
        float _y2[12] = {213.0, 270.0, 93.0, 416.0, 454.0, 456.0, 459.0, 294.0, 117.0, 156.0, 317.0, 390.0};
        for(int i=0; i<12; i++){
            x1[i] = _x1[i]; y1[i] = _y1[i]; x2[i] = _x2[i]; y2[i] = _y2[i];
        }
    }
    else{
        cout<<"This dataset still doesn't have given points."<<endl;
        exit(0);
    }


    Point2f p;
    for(int i = 0; i<12; i++){
        p.x = x1[i];
        p.y = y1[i];
        keypoints_left.push_back(p);
        p.x = x2[i];
        p.y = y2[i];
        keypoints_right.push_back(p);
    }
}

void tmp(){
    Mat x1 = Mat::ones(cv::Size(3,1), CV_64FC1);
    Mat x2_hat = Mat::ones(cv::Size(3,3), CV_64FC1);
    Mat tmp = x1 * x2_hat;
    cout<<"tmp: "<<tmp<<endl;
}

// 从dirpath目录下分别读取左右图像的路径，并存入left_image_paths和right_image_paths中
void getFilesList(String dirpath, vector<String> &left_image_paths, vector<String> &right_image_paths){
    // String left_dir = dirpath + "/image_02";
    // String right_dir = dirpath + "/image_03";
    struct dirent * entry;
    DIR *left_dir = opendir((dirpath + "/image_02/data/").c_str());
    DIR *right_dir = opendir((dirpath + "/image_03/data/").c_str());
    vector<String> left_images;
    vector<String> right_images;

    String name;
    while ((entry = readdir(left_dir)) != NULL){
        if(entry->d_type != DT_DIR){
            name = entry->d_name;
            left_images.push_back(name);
        }
    }

    while ((entry = readdir(right_dir)) != NULL){
        if(entry->d_type != DT_DIR){
            name = entry->d_name;
            right_images.push_back(name);
        }
    }

    sort(left_images.begin(), left_images.end());
    sort(right_images.begin(), right_images.end());

    assert(left_images.size() == right_images.size());

    bool PRINT_IMAGE_NAMES = 1;
    if(PRINT_IMAGE_NAMES){
        cout<<"\n left images: "<<endl;
        for(auto left_image:left_images) cout<<left_image<<endl;
        cout<<"\n right images: "<<endl;
        for(auto right_image:right_images) cout<<right_image<<endl;

    }
    for(int i = 0; i<left_images.size(); i++){
        assert(left_images[i] == right_images[i]);
        left_image_paths.push_back(dirpath + "/image_02/data/" + left_images[i]);
        right_image_paths.push_back(dirpath + "/image_03/data/" + right_images[i]);
    }

}