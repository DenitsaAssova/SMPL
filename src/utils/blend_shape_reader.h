#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <eigen3/Eigen/Dense>
#include <utility>
#include <jsoncpp/json/json.h>

using namespace Eigen;

// Generic function to read a matrix from a JSON file
void matrixReader(const std::string& file_path, MatrixXd& matrix, bool flat);

// Reads the beta(first 10 lines) and theta(rest 23 lines) blend shapes from a file;
bool betaThetaReader(
    const std::string &file_path,
    MatrixXd &s_beta,
    MatrixXd &s_theta
    );
