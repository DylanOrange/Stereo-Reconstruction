#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"

#include "VirtualSensor.h"


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
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

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

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();

		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		for(int h = 0; h < sensor.GetDepthImageHeight(); h++){
			for(int w = 0; w < sensor.GetDepthImageWidth(); w++){
				int idx = h * sensor.GetDepthImageWidth() + w;
				float depth = *(depthMap + idx);
				if(depth != MINF){
					float X_c = (float(w)-cX) * depth / fX;
					float Y_c = (float(h)-cY) * depth / fY;
					Vector4f P_c = Vector4f(X_c, Y_c, depth, 1);

					vertices[idx].position = trajectoryInv  * P_c;

					unsigned char R = *(colorMap + idx * 4    );
					unsigned char G = *(colorMap + idx * 4 + 1);
					unsigned char B = *(colorMap + idx * 4 + 2);
					unsigned char A = *(colorMap + idx * 4 + 3);
					vertices[idx].color = Vector4uc(R, G, B, A);
				}
				else{
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}
			}
		}


		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
