#include "include.h"
#include <fstream>

#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace std;
using namespace cv;


const String keys =
        "{help h usage ? |                  | print this message                                                }"
        "{num_keypoints  |40                | number of keypoints after filtering                               }"    
        "{detector       |orb               | keypoint detector: orb or sift                                    }";

int main(int argc, char*argv[]){

    CommandLineParser parser(argc, argv, keys);
    // parser.about("Disparity Filtering Demo");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    int num_keypoints = parser.get<int>("num_keypoints");

    String detector = parser.get<String>("detector");
    // int num_keypoints = 40;
    // if(argc == 2){
    //     num_keypoints = atoi(argv[1]);
    // }

    struct Dataset dataset;
    dataset.name = KITTI_2015;  // 选择所需要的数据集
    // dataset.name = MATLAB_TEST;
    dataset.rectified = 0;
    dataset.distort = 1;
    dataset.given_points = 0; // 手动给点还是用特征点检测

    // 对kitti数据集中的img2和img3文件夹进行遍历
    String dir_path = GetDirPath(dataset);
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    std::stringstream filename;
    filename << "rt_" << detector << "_" << num_keypoints <<".txt";
    // std::string filename = "rt.txt";
    std::ofstream outFile(filename.str(), ios::out | ios::app);
	if (!outFile.is_open()) return false;

    for(int i = 0; i<left_image_paths.size(); i++){
        
        String left = left_image_paths[i];
        String right = right_image_paths[i];

        // 读取图像
        Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
        if(IMAGE_SHOW){
            imshow("img_1", img_1);
            waitKey(0);
        }
        
        // // 如果读取的是undistorted和rectified的图像可以注销该部分
        if(dataset.distort == 1){
            Mat undistort_1, undistort_2;
            Undistort(img_1, img_2, undistort_1, undistort_2);
            img_1 = undistort_1;
            img_2 = undistort_2; // swallow copy or deep copy?
        }

        vector<Point2f> keypoints_left; 
        vector<Point2f> keypoints_right;
        // size_t num_keypoints = 40;

        if(dataset.given_points == 1){
            GetPoints(keypoints_left, keypoints_right, dataset);

            // show given points
            for(auto point:keypoints_left){
                cv::circle(img_1, point, 5, cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
                imshow("img_1", img_1);            
            }
            waitKey(0);
        }
        else{
            if(detector == "orb"){
                // 采用ORB进行特征点检测
                OrbDetector(img_1, img_2, keypoints_left, keypoints_right, (size_t)num_keypoints);
            }
            // else if(detector == "sift"){
            //     SiftDetector(img_1, img_2, keypoints_left, keypoints_right, (size_t)num_keypoints);
            // }
            else{
                cout<<"ERROR: kein keypoint detect method."<<endl;
                exit(0);
            }

            // if(DEBUG_PRINT) cout<<"keypoints_left.size(): "<<keypoints_left.size()<<endl;
        }        
                
        Mat fundamental_mat = findFundamentalMat(keypoints_left, keypoints_right,  cv::FM_8POINT);
        // cout<<"fundamental_mat: " << fundamental_mat << endl;
        Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);
        // cout<<"essential_mat: "<<essential_mat<<endl;

        // 由本质矩阵恢复出R和T, 注意这里的T仅表示方向，因为其L2-norm固定为1
        Mat R1, R2, T;
        decomposeEssentialMat(essential_mat, R1, R2, T);

        // 选择正确的R和T
        struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left, keypoints_right, dataset);
        if(DEBUG_PRINT){
            cout<<"R and t from eight-point method:"<<endl;
            cout<<"R: "<<transformation.R<<endl;
            cout<<"t: "<<transformation.t<<endl<<endl;
        }
        

        // 根据R和T对图像进行rectify
        if(!transformation.t.empty()){
            // Mat left_original = imread ( left, CV_LOAD_IMAGE_COLOR );
            // Mat right_original = imread ( right, CV_LOAD_IMAGE_COLOR );
            // Mat rectified_left, rectified_right;
            // Rectify_KITTI(transformation.R, transformation.t, left_original, right_original, rectified_left, rectified_right);

            outFile << transformation.R.at<double>(0,0) << " " << transformation.R.at<double>(0,1) << " " << transformation.R.at<double>(0,2) << " " 
                << transformation.R.at<double>(1,0) << " " << transformation.R.at<double>(1,1) << " " << transformation.R.at<double>(1,2) << " " 
                << transformation.R.at<double>(2,0) << " " << transformation.R.at<double>(2,1) << " " << transformation.R.at<double>(2,2) << std::endl;
            outFile << transformation.t.at<double>(0,0) << " " << transformation.t.at<double>(1,0) << " " << transformation.t.at<double>(2,0) << std::endl;

        }


        // 对rectify后的图像进行disparity计算


    
    }

	// close file
	outFile.close();



    return 0;
}
