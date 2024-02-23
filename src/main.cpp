#include "utils/blend_shape_reader.h"
#include "utils/io.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "smpl.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> test(Eigen::Matrix<T, Eigen::Dynamic, 1> t)
{
	return t;
}

int main(int argc, char **argv)
{
	// ---------------------------------
	// PARSE COMMAND-LINE ARGS
	// ---------------------------------

	bool regress_flag = false;
	bool help_flag = false;
	int opt;
	std::string PATH_BETA_THETA;

	if (argc < 2)
	{
		std::cerr << "Incorrect number of arguments!" << std::endl;
		printUsage();
		return 1;
	}
	else
	{
		PATH_BETA_THETA = argv[1];
	}

	while ((opt = getopt(argc, argv, "rh")) != -1)
	{
		switch (opt)
		{
		case 'r':
			regress_flag = true;
			break;
		case 'h':
			help_flag = true;
			break;
		default:
			std::cerr << "Use " << argv[0] << " -h to see the usage manual of the program." << std::endl;
			return 1;
		}
	}

	if (help_flag)
	{
		printUsage();
		return 0;
	}

	Eigen::MatrixXd s_beta = Eigen::Matrix<double, Dynamic, Dynamic>(10, 1);
	Eigen::MatrixXd s_theta = Eigen::Matrix<double, Dynamic, Dynamic>(24, 3);

	if (!betaThetaReader(PATH_BETA_THETA, s_beta, s_theta))
		return 1;

	// ---------------------------------
	// INITIALIZE THE SMPL MODEL
	// ---------------------------------

	SMPL smpl_model;

	// template vertices: (3N)
	std::string PATH_TEMPLATE_VERTICES =
		"../Data/smpl_female_vertices_reshaped.json";
	// blend shape basis: (3N, 10)
	std::string PATH_SHAPE_BLEND_SHAPE_BASIS =
		"../Data/smpl_female_shape_blend_shapes_reshaped.json";
	// pose shape basis: (3N, 207)
	std::string PATH_POSE_BLEND_SHAPE_BASIS =
		"../Data/smpl_female_pose_blend_shapes_reshaped.json";
	// joint_regressor: (24, 6890)
	std::string PATH_JOINT_REGRESSOR =
		"../Data/smpl_female_joint_regressor.json";
	// Weights: (6890, 24)
	std::string PATH_WEIGHTS =
		"../Data/smpl_female_weights.json";

	std::cout << "Loading SMPL params ..." << std::endl;
	smpl_model.loadDataFromFile(
		PATH_TEMPLATE_VERTICES,
		PATH_SHAPE_BLEND_SHAPE_BASIS,
		PATH_POSE_BLEND_SHAPE_BASIS,
		PATH_JOINT_REGRESSOR,
		PATH_WEIGHTS);
	std::cout << "Loading completed" << std::endl;

	// ---------------------------------
	// PERFROM SMPL FORWARD PASS
	// ---------------------------------

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> smpl_result;

	smpl_result = smpl_model.forward<double>(s_beta, s_theta, regress_flag);

	// ---------------------------------
	// SAVE MODEL
	// ---------------------------------

	std::ofstream file("smpl_point_cloud.txt");
	if (!file.is_open())
	{
		std::cerr << "Error saving point cloud" << std::endl;
		return 1;
	}
	for (int i = 0; i < smpl_result.rows(); i++)
		file << smpl_result(i, 0) << " " << smpl_result(i, 1) << " " << smpl_result(i, 2) << std::endl;
	file.close();

	std::cout << "Result saved" << std::endl;
	return 0;
}
