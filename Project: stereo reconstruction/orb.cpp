#include "include.h"

#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace std;
using namespace cv;

bool LessSort (DMatch a, DMatch b) { return (a.distance < b.distance); }

// 将vector中的KeyPoint转化为Point2f
void ConvertKeypointsVector(std::vector<KeyPoint>src, std::vector<cv::Point2f>& dst){
    Point2f point;
    for(int i = 0; i<src.size(); i++){
        point.x = src[i].pt.x;
        point.y = src[i].pt.y;
        dst.push_back(point);
    }
}

int OrbDetector (Mat img_1, Mat img_2, 
                 vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right, 
                 size_t num_keypoints = 10)
{

    //-- 读取图像
    // Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
    // Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    if(IMAGE_SHOW){
        // Mat outimg1;
        // drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        // imshow("ORB特征点",outimg1);
    }

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    vector<DMatch> good_matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );


    sort(matches.begin(), matches.end(), LessSort);

    if(DEBUG_PRINT){
        for(auto match : matches){
            cout<<match.distance<<endl;
        }
    }

    vector<KeyPoint> _keypoints_left, _keypoints_right;
    assert(matches.size() >= num_keypoints); 
    if(matches.size() < num_keypoints) num_keypoints = matches.size();
    for( size_t m = 0; m < num_keypoints; m++ ){
        int i1 = matches[m].queryIdx;
        int i2 = matches[m].trainIdx;
        _keypoints_left.push_back ( keypoints_1[i1] );
        _keypoints_right.push_back ( keypoints_2[i2] );
        good_matches.push_back(matches[m]);
    }

    if(IMAGE_SHOW){
        Mat outimg2_left, img_goodmatch;
        // drawKeypoints( img_1, keypoints_left, outimg2_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        // imshow("ORB特征点(前n个) left",outimg2_left);
        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
        imshow ( "优化后匹配点对", img_goodmatch );
        waitKey(0);
    }

    ConvertKeypointsVector(_keypoints_left, keypoints_left);
    ConvertKeypointsVector(_keypoints_right, keypoints_right);


    // //-- 第四步:匹配点对筛选
    // double min_dist=10000, max_dist=0;

    // //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     double dist = matches[i].distance;
    //     if ( dist < min_dist ) min_dist = dist;
    //     if ( dist > max_dist ) max_dist = dist;
    // }

    // printf ( "-- Max dist : %f \n", max_dist );
    // printf ( "-- Min dist : %f \n", min_dist );

    // //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    // std::vector< DMatch > good_matches;
    // for ( int i = 0; i < descriptors_1.rows; i++ )
    // {
    //     if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
    //     {
    //         good_matches.push_back ( matches[i] );
    //     }
    // }

    // //-- 第五步:绘制匹配结果
    // Mat img_match;
    // Mat img_goodmatch;
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    // imshow ( "所有匹配点对", img_match );
    // imshow ( "优化后匹配点对", img_goodmatch );
    

    return 0;
}
