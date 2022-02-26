#include"include.h"
#include <fstream>

#define DEBUG_PRINT 0
#define IMAGE_SHOW 1

using namespace cv;
using namespace std;

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;

	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	// float edgeThreshold = 0.01f; // 1cm
	float edgeThreshold = 0.10f; // 10 cm

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	unsigned nFaces = 0;
	std::vector<Vector3i> Face_idxs;
	for(int v = 0; v<height-1; v++){
		for(int u = 0; u<width-1; u++){
			int idx_0 =  v 		* width + u;
			int idx_1 = (v + 1) * width + u;
			int idx_2 =  v 		* width + u + 1;
			int idx_3 = (v + 1) * width + u + 1;

			bool valid_0 = (vertices[idx_0].position[0] != MINF);
			bool valid_1 = (vertices[idx_1].position[0] != MINF);
			bool valid_2 = (vertices[idx_2].position[0] != MINF);
			bool valid_3 = (vertices[idx_3].position[0] != MINF);

			if(valid_0 && valid_1 && valid_2){
				Vector4f p_0 = vertices[idx_0].position;
				Vector4f p_1 = vertices[idx_1].position;
				Vector4f p_2 = vertices[idx_2].position;
				float d_01 = (p_0-p_1).norm();
				float d_02 = (p_0-p_2).norm();
				float d_12 = (p_1-p_2).norm();
				if(d_01 < edgeThreshold && d_02 < edgeThreshold && d_12 < edgeThreshold)
				{	
					Vector3i Face_idx(idx_0, idx_1, idx_2);
					Face_idxs.push_back(Face_idx);
					nFaces++; 
				}
			}
			if(valid_3 && valid_1 && valid_2){
				Vector4f p_3 = vertices[idx_3].position;
				Vector4f p_1 = vertices[idx_1].position;
				Vector4f p_2 = vertices[idx_2].position;
				float d_31 = (p_3-p_1).norm();
				float d_32 = (p_3-p_2).norm();
				float d_12 = (p_1-p_2).norm();
				if(d_31 < edgeThreshold && d_32 < edgeThreshold && d_12 < edgeThreshold)
				{	
					Vector3i Face_idx(idx_1, idx_2, idx_3);
					Face_idxs.push_back(Face_idx);
					nFaces++; 
				}
			}
		}
	}


	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(int idx = 0; idx<nVertices; idx++){
		if((vertices+idx)->position[0] == MINF) outFile << "0.0 0.0 0.0 ";
		else
			outFile << (vertices+idx)->position[0] <<" "\
					<< (vertices+idx)->position[1] <<" "\
					<< (vertices+idx)->position[2] <<" ";
		
		outFile << int((vertices+idx)->color[0]) <<" "\
				<< int((vertices+idx)->color[1]) <<" "\
				<< int((vertices+idx)->color[2]) << " "\
				<< int((vertices+idx)->color[3]) <<std::endl;	
	}

	// TODO: save valid faces
	for(Vector3i& Face_idx:Face_idxs)	
		outFile<<"3 "<< Face_idx[0] << " " << Face_idx[1] << " "<< Face_idx[2] << " " << std::endl;	

	// close file
	outFile.close();

	return true;
}

void PointCloudGenerate(cv::Mat depth_map, cv::Mat rgb_map, struct Dataset dataset, int file_order = 0){

    if(dataset.name!=KITTI_TEST){
        cout<<"dataset error from pointcloud.cpp!"<<endl;
        exit(0);
    }
    float fX = 721.5377;
    float fY = 721.5377;
    float cX = 609.5593;
    float cY = 172.854;

    int width = depth_map.cols;
    int height = depth_map.rows;

    Vertex* vertices = new Vertex[width*height];
    for(int h = 0; h < height; h++){
        for(int w = 0; w < width; w++){
            int idx = h * width + w;
            // float depth = *(depthMap + idx);
            float depth = (float)(depth_map.at<short>(h,w)); 
            depth = depth;
            if(depth != MINF && depth != 0 && depth < 5000){ // range filter: (0, 50 meters)
                float X_c = (float(w)-cX) * depth / fX;
                float Y_c = (float(h)-cY) * depth / fY;
                Vector4f P_c = Vector4f(X_c, Y_c, depth, 1);

                vertices[idx].position = P_c;

                // opencv: bgr
                unsigned char R = rgb_map.at<Vec3b>(h,w)[2];
                unsigned char G = rgb_map.at<Vec3b>(h,w)[1];
                unsigned char B = rgb_map.at<Vec3b>(h,w)[0];
                unsigned char A = 255;
                vertices[idx].color = Vector4uc(R, G, B, A);
            }
            else{
                vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
                vertices[idx].color = Vector4uc(0, 0, 0, 0);
            }
        }
    }

    // write to the off file
    std::stringstream ss;
    ss << "kitti_test_" << file_order <<".off";
    WriteMesh(vertices, width, height, ss.str());

}