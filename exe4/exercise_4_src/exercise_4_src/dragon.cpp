#include "utils/io.h"
#include "utils/points.h"

#include "ceres/ceres.h"
#include <math.h>


// TODO: Implement the cost function
struct RegistrationCostFunction
{
	RegistrationCostFunction(const Point2D& point_1_, const Point2D& point_2_, const Weight& weight_)
		: point_1(point_1_), point_2(point_2_), weight(weight_)	{ }

	template<typename T>
	bool operator()(const T* const theta, const T* const tx, const T* const ty, T* residual) const
	{
		auto cos_theta = ceres::cos(theta[0]);
		auto sin_theta = ceres::sin(theta[0]);
		const T _x =  cos_theta * point_1.x - sin_theta * point_1.y + tx[0];
		const T _y =  sin_theta * point_1.x + cos_theta * point_1.y + ty[0];
		residual[0] = weight.w * (_x - point_2.x);
		residual[1] = weight.w * (_y - point_2.y);

		return true;
	}

private:
	Point2D point_1;
	Point2D point_2;
	Weight weight;
};


int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	// Read data points and the weights, and define the parameters of the problem
	const std::string file_path_1 = "../data/points_dragon_1.txt";
	const auto points1 = read_points_from_file<Point2D>(file_path_1);
	
	const std::string file_path_2 = "../data/points_dragon_2.txt";
	const auto points2 = read_points_from_file<Point2D>(file_path_2);
	
	const std::string file_path_weights = "../data/weights_dragon.txt";
	const auto weights = read_points_from_file<Weight>(file_path_weights);
	
	const double angle_initial = 0.0;
	const double tx_initial = 0.0;
	const double ty_initial = 0.0;
	
	double angle = angle_initial;
	double tx = tx_initial;
	double ty = ty_initial;

	ceres::Problem problem;

	// TODO: For each weighted correspondence create one residual block
	for (int i = 0; i<points1.size(); i++)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<RegistrationCostFunction, 2, 1, 1, 1>(
				new RegistrationCostFunction(points1[i], points2[i], weights[i].w)
			),
			nullptr, &angle, &tx, &ty
		);
	}	

	ceres::Solver::Options options;
	options.max_num_iterations = 25;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl;

	// Output the final values of the translation and rotation (in degree)
	std::cout << "Initial angle: " << angle_initial << "\ttx: " << tx_initial << "\tty: " << ty_initial << std::endl;
	std::cout << "Final angle: " << std::fmod(angle * 180 / M_PI, 360.0) << "\ttx: " << tx << "\tty: " << ty << std::endl;

	system("pause");
	return 0;
}
