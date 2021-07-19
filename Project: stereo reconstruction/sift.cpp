// #include "include.h"
// #include <opencv2/xfeatures2d.hpp>

// #define IMAGE_SHOW 0
// #define DEBUG_PRINT 0

// using namespace std;
// using namespace cv;
// using namespace features2d;
 

// bool LessSort_sift (DMatch a, DMatch b) { return (a.distance < b.distance); }

// void ConvertKeypointsVector_sift(std::vector<KeyPoint>src, std::vector<cv::Point2f>& dst){
//     Point2f point;
//     for(int i = 0; i<src.size(); i++){
//         point.x = src[i].pt.x;
//         point.y = src[i].pt.y;
//         dst.push_back(point);
//     }
// }

// int SiftDetector (Mat img_1, Mat img_2, 
//                  vector<Point2f>& keypoints_left, vector<Point2f>& keypoints_right, 
//                  size_t num_keypoints = 10)
// {	
	
// 	std::vector<KeyPoint> keypoints_1, keypoints_2;
//     Mat descriptors_1, descriptors_2;
// 	Ptr<SIFT> detector = SIFT::create();
// 	Ptr<SIFT> descriptor = SIFT::create();

// 	detector->detect (img_1, keypoints_1);
// 	detector->detect (img_2, keypoints_2);
// 	descriptor->compute (img_1, keypoints_1, descriptors_1);
// 	descriptor->compute (img_2, keypoints_2, descriptors_2);
// 	// siftPtr->detectAndCompute(img_1, keypoints_1, descriptors_1);
// 	// siftPtr->detectAndCompute(img_2,keypoints_2, descriptors_2);
// 	// Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create ("BruteForce-Hamming");
// 	BFMatcher matcher;

// 	vector<DMatch> matches;
// 	vector<DMatch> good_matches;
// 	// matcher->match (descriptors_1, descriptors_2, matches);
// 	matcher.match(descriptors_1, descriptors_2, matches);
// 	sort(matches.begin(), matches.end(), LessSort_sift);

// 	vector<KeyPoint> _keypoints_left, _keypoints_right;
//     assert(matches.size() >= num_keypoints); 
//     if(matches.size() < num_keypoints) num_keypoints = matches.size();
//     for( size_t m = 0; m < num_keypoints; m++ ){
//         int i1 = matches[m].queryIdx;
//         int i2 = matches[m].trainIdx;
//         if((keypoints_1[i1].pt.y - keypoints_2[i2].pt.y) > 50.0 || (keypoints_2[i2].pt.y - keypoints_1[i1].pt.y) > 50.0){  // 强行加入该限制条件
//             // m--;
//             continue;
//         }
//         if(DEBUG_PRINT){
//             cout<<"keypoints_1[i1].pt.y: "<<keypoints_1[i1].pt.y<<endl;
//             cout<<"keypoints_2[i2].pt.y: "<<keypoints_2[i2].pt.y<<endl;
//         }
//         _keypoints_left.push_back ( keypoints_1[i1] );
//         _keypoints_right.push_back ( keypoints_2[i2] );
//         good_matches.push_back(matches[m]);
//     }

// 	if(IMAGE_SHOW){
//         Mat img_goodmatch;
//         // drawKeypoints( img_1, keypoints_left, outimg2_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//         // imshow("ORB特征点(前n个) left",outimg2_left);
//         drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
//         imshow ( "优化后匹配点对", img_goodmatch );
//         waitKey(0);
//     }

// 	ConvertKeypointsVector_sift(_keypoints_left, keypoints_left);
//     ConvertKeypointsVector_sift(_keypoints_right, keypoints_right);

// 	return 0;
// }