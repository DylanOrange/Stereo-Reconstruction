#include "include.h"

#define DEBUG_PRINT 1
#define IMAGE_SHOW 1

using namespace std;
using namespace cv;

// Reference: https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html


int main(int argc, char*argv[]){

    int matching_method;
    if(argc > 1){
        if(strcmp(argv[1],"bm")==0) matching_method = BLOCK_MATCHING;
        else matching_method = SEMI_GLOBAL_MATCHING;
    }
    else
        matching_method = SEMI_GLOBAL_MATCHING;  // SEMI_GLOBAL_MATCHING or BLOCK_MATCHING

    

    struct Dataset dataset;
    dataset.name = KITTI_TEST; 
    // dataset.name = MATLAB_TEST;
    dataset.rectified = 1;
    dataset.distort = 0;
    dataset.given_points = 1; 
    dataset.baseline = 0.5327190420453419;
    dataset.focal_length = 721.5377;

    String dir_path = GetDirPath(dataset);
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    for(int i = 0; i<left_image_paths.size(); i++){
        String left = left_image_paths[i];
        String right = right_image_paths[i];

        Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
        if(IMAGE_SHOW){
            cout<<"img_1 type: "<<img_1.type()<<endl;
            imshow("img_1", img_1);
            waitKey(0);
        }

        int max_disp = 80; // the max disp determine the nearest distance
        int wsize = 9;
        Mat left_disp;

        if(matching_method == SEMI_GLOBAL_MATCHING){
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(5, max_disp, wsize);
            left_matcher->setP1(24 * wsize * wsize);  
            left_matcher->setP2(96 * wsize * wsize);  
            left_matcher->setMode(StereoSGBM::MODE_SGBM);
            // left_matcher->setDisp12MaxDiff(consistency); 
            // left_matcher->setUniquenessRatio(20);
            // left_matcher->setSpeckleRange(1);
            // left_matcher->setSpeckleWindowSize(2000);           
            // matching_time = (double)getTickCount();
            left_matcher-> compute(img_1, img_2, left_disp); 
        }
        else if(matching_method == BLOCK_MATCHING){
            Mat img_1_gray, img_2_gray;

            cv::cvtColor(img_1, img_1_gray, CV_BGR2GRAY);
            cv::cvtColor(img_2, img_2_gray, CV_BGR2GRAY);

            if(DEBUG_PRINT){
                cout<<"img_1_gray.type(): "<<img_1_gray.type()<<endl;
                cout<<"img_2_gray.type(): "<<img_2_gray.type()<<endl;
            }
            Ptr<StereoBM> bm = StereoBM::create(max_disp, wsize);
            bm->compute(img_1_gray, img_2_gray, left_disp);
        }

        if(DEBUG_PRINT){
            cout<<"left_disp.type(): "<<left_disp.type()<<endl;
            cout<<"left_disp.at<short>(38,226): "<<left_disp.at<short>(38,226)<<endl;
        }
        
        Mat depth;
        if(dataset.name == KITTI_TEST){
            
            depth = 16 * 100 * dataset.focal_length * dataset.baseline / left_disp;  // unit: cm
            cout<<"depth.type(): "<<depth.type()<<endl;
            cout<<"depth.at<short>(38,226): "<<depth.at<short>(38,226)<<endl;
            cout<<"depth.at<short>(40,226): "<<depth.at<short>(38,226)<<endl;
            // cv::imwrite("depth.png", depth);
        }
        else{
            cout<<"error dataset, now only support KITTI_TEST."<<endl;
            exit(0);
        }
        if(IMAGE_SHOW){
            Mat _left_disp_vis = left_disp / 16;
            Mat left_disp_vis;
            _left_disp_vis.convertTo(left_disp_vis, CV_8U);
            Mat _depth_vis, depth_vis;
            _depth_vis = depth/10;
            _depth_vis.convertTo(depth_vis, CV_8U);
            cv::imshow("left_disparity", left_disp_vis);
            
            cv::imwrite("left_disp_vis.png", left_disp_vis);

            // just for visualization of depth img
            for(int h = 0; h<depth_vis.rows; h++){
                for(int w = 0; w<depth_vis.cols; w++){
                    if(depth_vis.at<uchar>(h, w) == 255)
                        depth_vis.at<uchar>(h, w) = 0;
                }
            }

            cv::imshow("depth_vis (单位: 分米)", depth_vis);
            cv::imwrite("depth_vis_decimeter.jpg", depth_vis);  // unit: meters
            cv::waitKey(0);
        }
        

        PointCloudGenerate(depth, img_1, dataset, i);
    }

    return 0;
}