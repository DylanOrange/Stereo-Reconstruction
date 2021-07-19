// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>

#include "include.h"

#define IMAGE_SHOW 1
#define DEBUG_PRINT 0


using namespace std;
using namespace cv;

// 寻找两个图像中的对应点，像素坐标系
// 输入：img1, img2 两张图像
// 输出：points1, points2, 两组对应的2D点
int findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 );

// 相机内参
// double cx = 325.5;
// double cy = 253.5;
// double fx = 518.0;
// double fy = 519.0;

// // kitti rectify 后的内参
// double cx = 609.5593;
// double cy = 172.854;
// double fx = 721.5377;
// double fy = 721.5377;

// kitti rectify 前的内参
double cx = 6.960217e+02;
double cy = 2.241806e+02;
double fx = 9.597910e+02;
double fy = 9.569251e+02;

double cx2 = 6.957519e+02;
double cy2 = 2.242509e+02;
double fx2 = 9.037596e+02;
double fy2 = 9.019653e+02;

const String keys =
        "{help h usage ? |                  | print this message                                                }"
        "{num_keypoints    |40                 | number of keypoints after filtering                               }"    
        "{iter  |20               | number of optimization iterations                                      }"
        "{z  |10.0               | initial depth estimation                                      }"
        "{detector  |orb               | keypoint detector: orb or sift                                      }";


// bool LessSort (DMatch a, DMatch b) { return (a.distance < b.distance); }

// // 将vector中的KeyPoint转化为Point2f
// void ConvertKeypointsVector(std::vector<KeyPoint>src, std::vector<cv::Point2f>& dst){
//     Point2f point;
//     for(int i = 0; i<src.size(); i++){
//         point.x = src[i].pt.x;
//         point.y = src[i].pt.y;
//         dst.push_back(point);
//     }
// }


// int  findCorrespondingPoints( const cv::Mat& img_1, const cv::Mat& img_2, 
//                               vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 , int choose_match_num)
// {

//     //-- 初始化
//     std::vector<KeyPoint> keypoints_1, keypoints_2;
//     Mat descriptors_1, descriptors_2;
//     Ptr<FeatureDetector> detector = ORB::create();
//     Ptr<DescriptorExtractor> descriptor = ORB::create();
    
//     Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

//     //-- 第一步:检测 Oriented FAST 角点位置
//     detector->detect ( img_1,keypoints_1 );
//     detector->detect ( img_2,keypoints_2 );

//     //-- 第二步:根据角点位置计算 BRIEF 描述子
//     descriptor->compute ( img_1, keypoints_1, descriptors_1 );
//     descriptor->compute ( img_2, keypoints_2, descriptors_2 );

//     if(IMAGE_SHOW){
//         // Mat outimg1;
//         // drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//         // imshow("ORB特征点",outimg1);
//     }

//     //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
//     vector<DMatch> matches;
//     vector<DMatch> good_matches;
//     //BFMatcher matcher ( NORM_HAMMING );
//     matcher->match ( descriptors_1, descriptors_2, matches );


//     sort(matches.begin(), matches.end(), LessSort);

//     if(DEBUG_PRINT){
//         for(auto match : matches){
//             cout<<match.distance<<endl;
//         }
//     }

//     vector<KeyPoint> _keypoints_left, _keypoints_right;
//     int num_keypoints = choose_match_num;
//     cout<<"num_keypoints before filter: "<<matches.size()<<endl;
//     // assert(matches.size() >= num_keypoints); 
//     // if(matches.size() < num_keypoints) num_keypoints = matches.size();
//     for( size_t m = 0; m < num_keypoints; m++ ){
//         int i1 = matches[m].queryIdx;
//         int i2 = matches[m].trainIdx;
//         _keypoints_left.push_back ( keypoints_1[i1] );
//         _keypoints_right.push_back ( keypoints_2[i2] );
//         good_matches.push_back(matches[m]);
//     }

//     if(IMAGE_SHOW){
//         Mat outimg2_left, img_goodmatch;
//         // drawKeypoints( img_1, keypoints_left, outimg2_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//         // imshow("ORB特征点(前n个) left",outimg2_left);
//         drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
//         imshow ( "优化后匹配点对", img_goodmatch );
//         waitKey(0);
//     }

//     ConvertKeypointsVector(_keypoints_left, points1);
//     ConvertKeypointsVector(_keypoints_right, points2);

    
//     return true;
// }

void ReadTransformRecord(string filename, vector<Mat>& Rotations, vector<Mat>& translations){
    ifstream in;
    in.open(filename);
	string line;
    int flag = 0;
    double _R[9];
    double _t[3];

	while (getline(in, line)){//将in文件中的每一行字符读入到string line中
		stringstream ss(line);//使用string初始化stringstream
        Mat R = Mat::zeros(cv::Size(3,3), CV_64FC1);
        Mat t = Mat::zeros(cv::Size(1,3), CV_64FC1);
        double x;
        if(flag % 2 == 0){
            for(int i = 0; i<9; i++) ss >> _R[i];
            for(int i = 0; i<3; i++)
                for(int j = 0; j<3; j++)
                    R.at<double>(i, j) = _R[i*3+j];
            Rotations.push_back(R);
        }
        else{
            for(int i = 0; i<3; i++) ss >> _t[i];
            for(int i = 0; i<3; i++) t.at<double>(i, 0) = _t[i];
            translations.push_back(t);
        }
        flag++;
	}

}


// 使用方法： ./ba ${匹配点的数量} ${iteration} ${init_z}
// 选择 200+ 对匹配可以获得比较好的效果，默认为240对
// 目前仅仅支持在rectified后的KITTI图像上进行，如果更改数据集需要手动修改全局变量 cx, cy, fx, fy

int main( int argc, char** argv ){

    CommandLineParser parser(argc, argv, keys);
    if (parser.has("help")){
        parser.printMessage();
        return 0;
    }
    int num_keypoints = parser.get<int>("num_keypoints");
    int iter = parser.get<int>("iter");
    double z = parser.get<double>("z");

    String detector = parser.get<String>("detector");

    struct Dataset dataset;
    dataset.name = KITTI_2015;  // 选择所需要的数据集
    dataset.rectified = 0;
    dataset.distort = 1;
    dataset.given_points = 0; // 手动给点还是用特征点检测

    // 对kitti数据集中的img2和img3文件夹进行遍历
    String dir_path = GetDirPath(dataset);
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    std::stringstream filename;
    filename << "rt_" << detector << "_" << num_keypoints <<"_ba.txt";
    // std::string filename = "rt.txt";
    std::ofstream outFile(filename.str(), ios::out | ios::app);
	if (!outFile.is_open()) return false;

    string in_filename = "/home/dekai/3d_scanning/final/reconstruction/build/rt_orb_40.txt";
    vector<Mat> Rotations;
    vector<Mat> translations;
    ReadTransformRecord(in_filename, Rotations, translations);

    int record = 0; 

    for(int i_image = 0; i_image<left_image_paths.size(); i_image++){

        
        
        String left = left_image_paths[i_image];
        String right = right_image_paths[i_image];

        Mat img1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img2 = imread ( right, CV_LOAD_IMAGE_COLOR );

        // // 如果读取的是undistorted和rectified的图像可以注销该部分
        if(dataset.distort == 1){
            Mat undistort_1, undistort_2;
            Undistort(img1, img2, undistort_1, undistort_2);
            img1 = undistort_1;
            img2 = undistort_2; // swallow copy or deep copy?
        }


    cout<<"choose_match_num: "<<num_keypoints<<endl;

    vector<cv::Point2f> pts1, pts2;
    int num_keypoints_after_filter = 0;
    if(detector == "orb"){
        num_keypoints_after_filter = OrbDetector( img1, img2, pts1, pts2, num_keypoints);
    }
    // else if(detector == "sift"){
    //     num_keypoints_after_filter = SiftDetector( img1, img2, pts1, pts2, num_keypoints);
    // }
    else{
        cout<<"ERROR: kein keypoint detect method."<<endl;
        exit(0);
    }
    // if ( findCorrespondingPoints( img1, img2, pts1, pts2, num_keypoints) == false )
    // {
    //     cout<<"匹配点不够！"<<endl;
    //     return 0;
    // }
    // if(num_keypoints_after_filter == 0){
    //     cout<<"匹配点不够！"<<endl;
    //     return 0;
    // }
    cout<<"找到了"<<pts1.size()<<"组对应特征点。"<<endl;

    Mat fundamental_mat = findFundamentalMat(pts1, pts2,  cv::FM_8POINT);
    // cout<<"fundamental_mat: " << fundamental_mat << endl;
    Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);
    // cout<<"essential_mat: "<<essential_mat<<endl;

    // 由本质矩阵恢复出R和T, 注意这里的T仅表示方向，因为其L2-norm固定为1
    Mat R1, R2, T;
    decomposeEssentialMat(essential_mat, R1, R2, T);

    // 选择正确的R和T
    Mat lambda = Mat::zeros(cv::Size(1, num_keypoints), CV_64FC1);
    struct transformation transformation = RecoverRT_2(R1, R2, T, pts1, pts2, dataset, lambda);
    // cout<<"lambda: "<<lambda<<endl;

    cout<<"R: "<<transformation.R<<endl;
    cout<<"t: "<<transformation.t<<endl;
    // system("pause");
    if(!transformation.R.empty()){

    

    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // 这里是9还是6？
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);



    // 添加节点
    // 两个位姿节点
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        if(i == 0){
            v->setEstimate( g2o::SE3Quat() );
        }
        else{
            g2o::Vector3 _t;
            // _t << -0.5268885591265065, 0.03775366888076277, -0.06893971645823953;
            //  _t << -0.4883209864189943, 0.02270631091623175, 0.2116993514989185;
            _t << translations[record].at<double>(0,0),translations[record].at<double>(1,0),translations[record].at<double>(2,0);
            g2o::Matrix3 _R;
            // _R << 0.9996847321281889, 0.02333824274224507, -0.009260819374934438,
            //     -0.02359096283361033, 0.9993241901599044, -0.02818917228005423,
            //     0.008596675076698639, 0.02839875678538437, 0.9995597069663573;

            // _R << 0.9996822443615183, 0.01291140343696726, -0.02164961823232908, 
            //     -0.01246159322056104, 0.999706312018693, 0.02078457130635199,
            //     0.02191161798505531, -0.02050817815583449, 0.9995495463587611;
            _R << Rotations[record].at<double>(0, 0),Rotations[record].at<double>(0, 1),Rotations[record].at<double>(0, 2),
                  Rotations[record].at<double>(1, 0),Rotations[record].at<double>(1, 1),Rotations[record].at<double>(1, 2),
                  Rotations[record].at<double>(2, 0),Rotations[record].at<double>(2, 1),Rotations[record].at<double>(2, 2);
            record++;
            // _R = g2o::Matrix3::Identity();
            v->setEstimate(g2o::SE3Quat(_R, _t));
        }
        
        optimizer.addVertex( v );
    }
    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        // 由于深度不知道，只能把深度设置为1了
        // z = lambda.at<double>(i);
        double z = 10.0;

        double x = ( pts1[i].x - cx ) * z / fx; 
        double y = ( pts1[i].y - cy ) * z / fy; 
        v->setMarginalized(true);
        v->setEstimate( Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }

    // 准备相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );

    g2o::CameraParameters* camera2 = new g2o::CameraParameters( fx2, Eigen::Vector2d(cx2, cy2), 0 );
    camera2->setId(1);
    optimizer.addParameter( camera2 );


    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,1);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(iter);
    cout<<"优化完毕"<<endl;

    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;
    cout<<"translation: "<<pose.translation()/pose.translation().norm()*0.5327190420453419<<endl;

    Eigen::Vector3d translation = pose.translation()/pose.translation().norm()*0.5327190420453419;

    outFile << pose.rotation()(0,0) << " " << pose.rotation()(0,1) << " " << pose.rotation()(0,2) << " " <<
               pose.rotation()(1,0) << " " << pose.rotation()(1,1) << " " << pose.rotation()(1,2) << " " <<
               pose.rotation()(2,0) << " " << pose.rotation()(2,1) << " " << pose.rotation()(2,2) << std::endl;
    
    outFile << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;

    // 以及所有特征点的位置
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        // cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        // cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }
    
    // 估计inlier的个数
    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 )
        {
            // cout<<"error = "<<e->chi2()<<endl;
        }
        else 
        {
            inliers++;
        }
    }
    
    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");

    // return 0;

    }
    }
    outFile.close();
    return 0; 
}


