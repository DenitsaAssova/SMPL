#include "blend_shape_reader.h"

using namespace Eigen;

// Generic function to read a matrix from a JSON file
void matrixReader(const std::string &file_path, MatrixXd &matrix, bool flat)
{
	Json::Value root;
	std::ifstream config_doc(file_path, std::ifstream::binary);
	config_doc >> root;

	for (unsigned int i = 0; i < root.size(); i++)
	{
		if (!flat)
		{
			// Handle the case where the JSON structure is an array of values
			for (unsigned int j = 0; j < root[i].size(); ++j)
			{
				matrix(i, j) = root[i][j].asDouble();
			}
		}
		else
		{
			// Handle the case where the JSON structure is a flat array
			matrix(i, 0) = root[i].asDouble();
		}
	}
}

// Reads the beta(first 10 lines) and theta(rest 23 lines) blend shapes from a file;
bool betaThetaReader(const std::string &file_path, MatrixXd &s_beta, MatrixXd &s_theta)
{
	std::ifstream file(file_path);

	if (!file.is_open())
	{
		std::cerr << "Error opening file: " << file_path << std::endl;
		return false;
	}

	// Read beta values
	for (int i = 0; i < 10; ++i)
	{
		if (!(file >> s_beta(i, 0)))
		{
			std::cerr << "Error reading beta values." << std::endl;
			return false;
		}
	}

	// Read theta values
	for (int i = 0; i < 23; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			if (!(file >> s_theta(i, j)))
			{
				std::cerr << "Error reading theta values." << std::endl;
				return false;
			}
		}
	}

	file.close();
	return true;
}