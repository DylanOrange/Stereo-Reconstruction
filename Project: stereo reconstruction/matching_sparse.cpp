#include "include.h"
#include <fstream>

#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace std;
using namespace cv;


const String keys =
        "{help h usage ? |     | print this message                  }"
        "{num_keypoints  | 40  | number of keypoints after filtering }"    
        "{detector       | orb | keypoint detector: orb or sift      }";

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

    struct Dataset dataset;
    dataset.name = KITTI_2015;  // choose the dataset
    // dataset.name = MATLAB_TEST;
    dataset.rectified = 0;
    dataset.distort = 1;
    dataset.given_points = 0; // directly give the match points or detect them with detector

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

        Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
        if(IMAGE_SHOW){
            imshow("img_1", img_1);
            waitKey(0);
        }
        
        if(dataset.distort == 1){
            Mat undistort_1, undistort_2;
            Undistort(img_1, img_2, undistort_1, undistort_2);
            img_1 = undistort_1;
            img_2 = undistort_2; // swallow copy or deep copy?
        }

        vector<Point2f> keypoints_left; 
        vector<Point2f> keypoints_right;

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
                OrbDetector(img_1, img_2, keypoints_left, keypoints_right, (size_t)num_keypoints);
            }
            else if(detector == "sift"){
                SiftDetector(img_1, img_2, keypoints_left, keypoints_right, (size_t)num_keypoints);
            }
            else{
                cout<<"ERROR: kein keypoint detect method."<<endl;
                exit(0);
            }
        }        
                
        Mat fundamental_mat = findFundamentalMat(keypoints_left, keypoints_right,  cv::FM_8POINT);
        // cout<<"fundamental_mat: " << fundamental_mat << endl;
        Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);
        // cout<<"essential_mat: "<<essential_mat<<endl;

        Mat R1, R2, T;
        decomposeEssentialMat(essential_mat, R1, R2, T);

        // Choose the correct R and t
        struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left, keypoints_right, dataset);
        if(DEBUG_PRINT){
            cout<<"R and t from eight-point method:"<<endl;
            cout<<"R: "<<transformation.R<<endl;
            cout<<"t: "<<transformation.t<<endl<<endl;
        }
        
        // record the R and t
        if(!transformation.t.empty()){

            outFile << transformation.R.at<double>(0,0) << " " << transformation.R.at<double>(0,1) << " " << transformation.R.at<double>(0,2) << " " 
                    << transformation.R.at<double>(1,0) << " " << transformation.R.at<double>(1,1) << " " << transformation.R.at<double>(1,2) << " " 
                    << transformation.R.at<double>(2,0) << " " << transformation.R.at<double>(2,1) << " " << transformation.R.at<double>(2,2) << std::endl;
            outFile << transformation.t.at<double>(0,0) << " " << transformation.t.at<double>(1,0) << " " << transformation.t.at<double>(2,0) << std::endl;

        }    
    }
	outFile.close();
    return 0;
}
