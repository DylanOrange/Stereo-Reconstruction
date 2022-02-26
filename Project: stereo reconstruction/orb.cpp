#include "include.h"

#define DEBUG_PRINT 0
#define IMAGE_SHOW 0

using namespace std;
using namespace cv;

bool LessSort (DMatch a, DMatch b) { return (a.distance < b.distance); }

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


    // init
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    if(IMAGE_SHOW){
        // Mat outimg1;
        // drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        // imshow("ORB特征点",outimg1);
    }

    vector<DMatch> matches;
    vector<DMatch> good_matches;
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
        imshow ( "good match", img_goodmatch );
        waitKey(0);
    }

    ConvertKeypointsVector(_keypoints_left, keypoints_left);
    ConvertKeypointsVector(_keypoints_right, keypoints_right);

    return 0;
}
