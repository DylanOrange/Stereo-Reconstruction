/**
 * References: https://www.cnblogs.com/gaoxiang12/p/5304272.html
 *             https://github.com/gaoxiang12/g2o_ba_example
 */

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


// left camera
double cx = 6.960217e+02;
double cy = 2.241806e+02;
double fx = 9.597910e+02;
double fy = 9.569251e+02;

// right camera
double cx2 = 6.957519e+02;
double cy2 = 2.242509e+02;
double fx2 = 9.037596e+02;
double fy2 = 9.019653e+02;

double baseline = 0.5327190420453419;

const String keys =
        "{ help h usage ? |      | print this message                   }"
        "{ num_keypoints  | 40   | number of keypoints after filtering  }"    
        "{ iter           | 20   | number of optimization iterations    }"
        "{ z              | 10.0 | initial depth estimation             }"
        "{ detector       | orb  | keypoint detector: orb or sift       }";



void ReadTransformRecord(string filename, vector<Mat>& Rotations, vector<Mat>& translations){
    ifstream in;
    in.open(filename);
	string line;
    int flag = 0;
    double _R[9];
    double _t[3];

	while (getline(in, line)){
		stringstream ss(line);
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


// Usage ./ba --detector=${detector method e.g. SIFT or ORB} --num_keypoints=${num_keypoints} --iter=${iteration times of optimization}

int main( int argc, char** argv ){

    CommandLineParser parser(argc, argv, keys);
    if(parser.has("help")){
        parser.printMessage();
        return 0;
    }
    int num_keypoints = parser.get<int>("num_keypoints");
    int iter = parser.get<int>("iter");
    double z = parser.get<double>("z");

    String detector = parser.get<String>("detector");

    struct Dataset dataset;
    dataset.name = KITTI_2015;  // choose the dataset
    dataset.rectified = 0;
    dataset.distort = 1;
    dataset.given_points = 0; // give the point manually or detect them with sift or orb

    String dir_path = GetDirPath(dataset);
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    std::stringstream filename;
    filename << "rt_" << detector << "_" << num_keypoints <<"_ba.txt";
    std::ofstream outFile(filename.str(), ios::out | ios::app);
	if (!outFile.is_open()) return false;

    string in_filename = "../build/rt_orb_40.txt";  // set the path of record txt of R and t from eight-point algorithm
    vector<Mat> Rotations;
    vector<Mat> translations;
    ReadTransformRecord(in_filename, Rotations, translations);

    int record = 0; 

    for(int i_image = 0; i_image<left_image_paths.size(); i_image++){
        
        String left = left_image_paths[i_image];
        String right = right_image_paths[i_image];

        Mat img1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img2 = imread ( right, CV_LOAD_IMAGE_COLOR );

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
        else if(detector == "sift"){
            num_keypoints_after_filter = SiftDetector( img1, img2, pts1, pts2, num_keypoints);
        }
        else{
            cout<<"ERROR: kein keypoint detect method."<<endl;
            exit(0);
        }

        cout<<"find "<<pts1.size()<<" pairs of feature points."<<endl;

        Mat fundamental_mat = findFundamentalMat(pts1, pts2,  cv::FM_8POINT);
        Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);

        Mat R1, R2, T;
        decomposeEssentialMat(essential_mat, R1, R2, T);

        // choose the correct R and T
        Mat lambda = Mat::zeros(cv::Size(1, num_keypoints), CV_64FC1);
        struct transformation transformation = RecoverRT_2(R1, R2, T, pts1, pts2, dataset, lambda);

        cout<<"R: "<<transformation.R<<endl;
        cout<<"t: "<<transformation.t<<endl;
        // system("pause");
        if(!transformation.R.empty()){

            g2o::SparseOptimizer optimizer;
            typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
            typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

            auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
            optimizer.setAlgorithm(solver);
            optimizer.setVerbose(true);

            // Add nodes
            // Add two nodes of pose
            for ( int i=0; i<2; i++ )
            {
                g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
                v->setId(i);
                if (i == 0)
                    v->setFixed( true ); // fix the first point
                if(i == 0){
                    v->setEstimate( g2o::SE3Quat() );
                }
                else{
                    // get the R and t from record txt
                    g2o::Vector3 _t;
                    // _t << -0.5268885591265065, 0.03775366888076277, -0.06893971645823953;
                    //  _t << -0.4883209864189943, 0.02270631091623175, 0.2116993514989185;
                    _t << translations[record].at<double>(0,0),translations[record].at<double>(1,0),translations[record].at<double>(2,0);
                    
                    g2o::Matrix3 _R;
                    // _R << 0.9996847321281889, 0.02333824274224507, -0.009260819374934438,
                    //     -0.02359096283361033, 0.9993241901599044, -0.02818917228005423,
                    //     0.008596675076698639, 0.02839875678538437, 0.9995597069663573;

                    _R << Rotations[record].at<double>(0, 0),Rotations[record].at<double>(0, 1),Rotations[record].at<double>(0, 2),
                        Rotations[record].at<double>(1, 0),Rotations[record].at<double>(1, 1),Rotations[record].at<double>(1, 2),
                        Rotations[record].at<double>(2, 0),Rotations[record].at<double>(2, 1),Rotations[record].at<double>(2, 2);
                    record++;
                    v->setEstimate(g2o::SE3Quat(_R, _t));
                }
                
                optimizer.addVertex( v );
            }

            // Add the nodes of feature points
            for ( size_t i=0; i<pts1.size(); i++ )
            {
                g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
                v->setId( 2 + i );
                // here we just set the initial depth to 10 m
                double z = 10.0;
                double x = ( pts1[i].x - cx ) * z / fx; 
                double y = ( pts1[i].y - cy ) * z / fy; 
                v->setMarginalized(true);
                v->setEstimate( Eigen::Vector3d(x,y,z) );
                optimizer.addVertex( v );
            }

            // set the intrinsic of cameras
            g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
            camera->setId(0);
            optimizer.addParameter( camera );

            g2o::CameraParameters* camera2 = new g2o::CameraParameters( fx2, Eigen::Vector2d(cx2, cy2), 0 );
            camera2->setId(1);
            optimizer.addParameter( camera2 );


            // setup the edge information
            // the edge in first frame
            vector<g2o::EdgeProjectXYZ2UV*> edges;
            for ( size_t i=0; i<pts1.size(); i++ )
            {
                g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
                edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
                edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
                edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
                edge->setInformation( Eigen::Matrix2d::Identity() );
                edge->setParameterId(0, 0);
                edge->setRobustKernel( new g2o::RobustKernelHuber() );
                optimizer.addEdge( edge );
                edges.push_back(edge);
            }
            // the edge in second frame
            for ( size_t i=0; i<pts2.size(); i++ )
            {
                g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
                edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
                edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
                edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
                edge->setInformation( Eigen::Matrix2d::Identity() );
                edge->setParameterId(0,1);
                edge->setRobustKernel( new g2o::RobustKernelHuber() );
                optimizer.addEdge( edge );
                edges.push_back(edge);
            }

            cout<<"begin to optimize"<<endl;
            optimizer.setVerbose(true);
            optimizer.initializeOptimization();
            optimizer.optimize(iter);
            cout<<"the optimization finish"<<endl;

            // print R and t, note that the length of t here need to be reset to baseline
            g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
            Eigen::Isometry3d pose = v->estimate();
            cout<<"Pose: "<<endl<<pose.matrix()<<endl;
            cout<<"translation: "<<endl<<pose.translation() / pose.translation().norm() * baseline<<endl;

            Eigen::Vector3d translation = pose.translation()/pose.translation().norm()*baseline;

            outFile << pose.rotation()(0,0) << " " << pose.rotation()(0,1) << " " << pose.rotation()(0,2) << " " <<
                    pose.rotation()(1,0) << " " << pose.rotation()(1,1) << " " << pose.rotation()(1,2) << " " <<
                    pose.rotation()(2,0) << " " << pose.rotation()(2,1) << " " << pose.rotation()(2,2) << std::endl;
            
            outFile << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;

            // the position of all feature points
            for ( size_t i=0; i<pts1.size(); i++ )
            {
                g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
                // cout<<"vertex id "<<i+2<<", pos = ";
                Eigen::Vector3d pos = v->estimate();
                // cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
            }
            
            // the number of inliers
            int inliers = 0;
            for ( auto e:edges )
            {
                e->computeError();
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


