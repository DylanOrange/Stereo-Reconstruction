#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);
		
		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements

		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
		for (Vector3f point : points){
			mean += point;
		}
		mean /= points.size();
        return mean;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm.
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		// Important: The covariance matrices should contain mean-centered source/target points.

		// 先得到 X 和 Y 矩阵
		int n_points = sourcePoints.size();
		MatrixXf X(3, n_points);	// 默认初始值为0
		MatrixXf Y(3, n_points);
		for(int i=0; i<n_points; i++){
			X.col(i) += sourcePoints[i];
			Y.col(i) += targetPoints[i];
		}

		// 得到 _X 和 _Y
		MatrixXf _X = X;
		MatrixXf _Y = Y;
		for(int i = 0; i < n_points; i++){
			_X.col(i) -= sourceMean;
			_Y.col(i) -= targetMean;
		}		
		
		// 采用优化的方法得到 R, which满足 ||_Y - R * _X||^2_F -> min
		JacobiSVD<MatrixXf> svd(_Y * _X.transpose(), ComputeThinU | ComputeThinV);
		const MatrixXf& U = svd.matrixU();
		const MatrixXf& V = svd.matrixV();
		Matrix3f R = U * V.transpose();
		if(R.determinant() == -1){
			Matrix3f N;
			N << 1, 0,  0, 
				 0, 1,  0, 
				 0, 0, -1;
			R = U * N * V.transpose();
		}
		
        return R;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target points.
		
        Vector3f translation = Vector3f::Zero();
		translation = targetMean - rotation * sourceMean;
        return translation;
	}
};