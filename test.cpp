#include "include.h"
#include <fstream>

#define DEBUG_PRINT 1
#define IMAGE_SHOW 1

using namespace std;
using namespace cv;


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

int main(int argc, char*argv[]){

    string filename = "../build/rt_sift_40.txt";
    vector<Mat> Rotations;
    vector<Mat> translations;
    ReadTransformRecord(filename, Rotations, translations);
    for(auto R:Rotations){
        cout<<R<<endl;
        cout<<"###"<<endl;
    }
    cout<<"***"<<endl;
    for(auto t:translations){
        cout<<t<<endl;
        cout<<"###"<<endl;
    }

    return 0;
}