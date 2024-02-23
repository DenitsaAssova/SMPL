#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <vector>
#include "utils/blend_shape_reader.h"
#include <iostream>

using namespace Eigen;

template <typename T>
struct TransformM
{
	TransformM() = default;

	TransformM(Matrix<T, Dynamic, Dynamic> t_) : t{t_}
	{
	}

	Matrix<T, Dynamic, Dynamic> t{};
};

class SMPL
{
public:
	Matrix<double, Dynamic, Dynamic> s_template_vertices;
	Matrix<double, Dynamic, Dynamic> s_shape_blend_shape_basis;
	Matrix<double, Dynamic, Dynamic> s_pose_blend_shape_basis;
	Matrix<double, Dynamic, Dynamic> s_joint_regressor;
	Matrix<double, Dynamic, Dynamic> s_weights;
	Matrix<double, Dynamic, Dynamic> s_parents;

	//Matrix<double, Dynamic, Dynamic> shaped_template_joints;

	//Constructors
	SMPL()
	{
		s_template_vertices = MatrixXd(20670, 1);
		s_shape_blend_shape_basis = MatrixXd(20670, 10);
		s_pose_blend_shape_basis = MatrixXd(20670, 207);
		s_joint_regressor = MatrixXd(24, 6890);
		s_weights = MatrixXd(6890, 24);
		s_parents = MatrixXd(23, 1);
	}

	void loadDataFromFile(
		std::string &PATH_TEMPLATE_VERTICES,
		std::string &PATH_SHAPE_BLEND_SHAPE_BASIS,
		std::string &PATH_POSE_BLEND_SHAPE_BASIS,
		std::string &PATH_JOINT_REGRESSOR,
		std::string &PATH_WEIGHTS)
	{
		try
		{
			//template vertices: (3N)
			matrixReader(PATH_TEMPLATE_VERTICES, s_template_vertices, true);
			//blend shape basis: (3N, 10)
			matrixReader(PATH_SHAPE_BLEND_SHAPE_BASIS, s_shape_blend_shape_basis, false);
			//pose shape basis: (3N, 207)
			matrixReader(PATH_POSE_BLEND_SHAPE_BASIS, s_pose_blend_shape_basis, false);
			//joint_regressor: (24, 6890)
			matrixReader(PATH_JOINT_REGRESSOR, s_joint_regressor, false);
			//Weights: (6890, 24)
			matrixReader(PATH_WEIGHTS, s_weights, false);
		}
		catch (Json::RuntimeError)
		{
			std::cerr << "Failed to read SMPL params" << std::endl;
		}
	}

	//Setters
	void setTemplateVertices(Matrix<double, Dynamic, Dynamic> template_vertices)
	{
		s_template_vertices = template_vertices;
	}
	void setShapeBlendShapeBasis(Matrix<double, Dynamic, Dynamic> shape_blend_shape_basis)
	{
		s_shape_blend_shape_basis = shape_blend_shape_basis;
	}
	void setPoseBlendShapeBasis(Matrix<double, Dynamic, Dynamic> pose_blend_shape_basis)
	{
		s_pose_blend_shape_basis = pose_blend_shape_basis;
	}
	void setJointRegressor(Matrix<double, Dynamic, Dynamic> joint_regressor)
	{
		s_joint_regressor = joint_regressor;
	}
	void setWeights(Matrix<double, Dynamic, Dynamic> weights)
	{
		s_weights = weights;
	}

	void setParents(Matrix<double, Dynamic, Dynamic> parents)
	{
		s_parents = parents;
	}
	class
		//Getters
		Matrix<double, Dynamic, Dynamic>
		getTemplateVertices()
	{
		return s_template_vertices;
	} Matrix<double, Dynamic, Dynamic> getShapeBlendShapeBasis()
	{
		return s_shape_blend_shape_basis;
	}
	Matrix<double, Dynamic, Dynamic> getPoseBlendShapeBasis()
	{
		return s_pose_blend_shape_basis;
	}
	Matrix<double, Dynamic, Dynamic> getJointRegressor()
	{
		return s_joint_regressor;
	}
	Matrix<double, Dynamic, Dynamic> getWeights()
	{
		return s_weights;
	}

	Matrix<double, Dynamic, Dynamic> getParents()
	{
		return s_parents;
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> rodrigues(Matrix<T, Dynamic, 1> theta)
	{
		Matrix<T, Dynamic, Dynamic> rotation(3, 3);
		Matrix<T, Dynamic, 1> axis(3);

		T angle = theta.norm();
		if (angle != 0)
		{
			for (unsigned int i = 0; i < theta.size(); i++)
			{

				axis(i) = theta(i) / angle;
			}

			T cosine, sine;
			cosine = cos(angle);
			sine = sin(angle);

			//the skew symmetric matrix of the 3-vector 'axis'
			Matrix<T, Dynamic, Dynamic> K(3, 3);

			K << 0, -axis(2), axis(1),
				axis(2), 0, -axis(0),
				-axis(1), axis(0), 0;

			//Identity matrix
			Matrix<T, Dynamic, Dynamic> ident = Matrix<T, Dynamic, Dynamic>::Identity(3, 3);

			rotation = ident + sine * K + (1 - cosine) * K * K;

			return rotation;
		}
		else
			return Matrix<T, 3, 3>::Identity(3, 3);
	}

	template <typename T>
	Matrix<T, Dynamic, 1> shapeBlend(Matrix<T, Dynamic, Dynamic> beta)
	{
		Matrix<T, Dynamic, 1> displacement(20670);
		//Matrix<T, Dynamic, Dynamic> beta = getBeta();
		Matrix<T, Dynamic, Dynamic> shape_blend_shape_basis = getShapeBlendShapeBasis();

		for (unsigned int i = 0; i < displacement.size(); i++)
		{

			displacement(i) = 0;
		}

		for (unsigned int j = 0; j < beta.size(); j++)
		{
			Matrix<T, Dynamic, 1> temp(20670);

			for (unsigned int i = 0; i < displacement.size(); i++)
			{

				temp(i) = beta(j) * shape_blend_shape_basis(i, j);
			}

			displacement = displacement + temp;
		}
		return displacement;
	}

	template <typename T>
	Matrix<T, Dynamic, 1> poseBlend(Matrix<T, Dynamic, Dynamic> theta)
	{
		Matrix<T, Dynamic, 1> pose_offset(20670);
		//Matrix<T, Dynamic, Dynamic> theta = getTheta();
		Matrix<T, Dynamic, Dynamic> pose_blend_shape_basis = getPoseBlendShapeBasis();

		for (unsigned int i = 0; i < pose_offset.size(); i++)
		{

			pose_offset(i) = 0;
		}

		//Map the rest pose vector to a vector of concatenated part relative rotation matrices (with rodrigues' formula)
		Matrix<T, Dynamic, 1> rest_pose_rotation(207);
		for (unsigned int i = 0; i < 207; i += 9)
		{

			rest_pose_rotation.segment(i, 9) << 1, 0, 0,
				0, 1, 0,
				0, 0, 1;
		}

		Matrix<T, Dynamic, 1> pose_rotation(207);

		//Compute rotation matrix of theta with rodrigues
		//Theta is (24,3) but we need only 23 joints, remove base joint at first position

		int counter = 0;

		for (unsigned int i = 1; i < theta.rows(); i++)
		{
			Matrix<T, Dynamic, Dynamic> rotation(3, 3);

			//3x3 rotation matrix
			rotation = rodrigues<T>(theta.row(i));
			//print_joints(rotation);

			//unroll rotation matrix
			pose_rotation.segment(counter, 9) << rotation(0, 0), rotation(0, 1), rotation(0, 2),
				rotation(1, 0), rotation(1, 1), rotation(1, 2),
				rotation(2, 0), rotation(2, 1), rotation(2, 2);

			counter += 9;
		}

		for (unsigned int j = 0; j < (theta.rows() - 1) * theta.cols(); j++)
		{
			Matrix<T, Dynamic, 1> temp(pose_offset.size());
			//R(theta) - R(rest_theta))
			double r_diff = pose_rotation(j) - rest_pose_rotation(j);
			//std::cout<<"r_diff: "<<r_diff<<std::endl;
			for (unsigned int i = 0; i < pose_offset.size(); i++)
			{

				temp(i) = r_diff * pose_blend_shape_basis(i, j);
			}

			pose_offset = pose_offset + temp;
		}

		return pose_offset;
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> reshape_vector_to_matrix(
		Matrix<T, Dynamic, 1> vector)
	{

		Matrix<T, Dynamic, Dynamic> matrix(6890, 3);

		int j = 0;

		for (unsigned int i = 0; i < vector.size(); i += 3)
		{
			matrix.row(j) = (vector.segment(i, 3)).transpose();
			j++;
		}

		return matrix;
	}

	template <typename T>
	void blending(Matrix<T, Dynamic, Dynamic> &reshaped_shaped_vertices,
				  Matrix<T, Dynamic, Dynamic> &reshaped_posed_vertices, Matrix<T, Dynamic, Dynamic> beta, Matrix<T, Dynamic, Dynamic> theta)
	{

		Matrix<T, Dynamic, 1> template_vertices = getTemplateVertices();
		Matrix<T, Dynamic, 1> displacement(20670);
		displacement = shapeBlend<T>(beta);

		Matrix<T, Dynamic, 1> pose_offset(20670);
		pose_offset = poseBlend<T>(theta);

		Matrix<T, Dynamic, 1> shaped_template_vertices(20670);
		Matrix<T, Dynamic, 1> posed_template_vertices(20670);

		for (unsigned int i = 0; i < template_vertices.size(); i++)
		{
			shaped_template_vertices(i) = template_vertices(i) + displacement(i);
			posed_template_vertices(i) = shaped_template_vertices(i) + pose_offset(i);
		}

		reshaped_shaped_vertices = reshape_vector_to_matrix(shaped_template_vertices);
		reshaped_posed_vertices = reshape_vector_to_matrix(posed_template_vertices);
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> regressJoints(
		Matrix<T, Dynamic, Dynamic> vertices)
	{

		Matrix<T, Dynamic, Dynamic> shaped_template_joints = s_joint_regressor * vertices;
		return shaped_template_joints;
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> build_rotation_matrix(
		Matrix<T, Dynamic, Dynamic> theta, Matrix<T, Dynamic, 1> joint,
		int jointID)
	{

		Matrix<T, Dynamic, Dynamic> rotation_matrix = Matrix<T, 4, 4>::Identity();

		Matrix<T, Dynamic, Dynamic> rotation(3, 3);

		//3x3 rotation matrix of joint
		rotation = rodrigues<T>(theta.row(jointID));
		rotation_matrix.block(0, 0, 3, 3) = rotation;
		rotation_matrix.block(0, 3, 3, 1) = joint;

		return rotation_matrix;
	}

	template <typename T>
	void rigid_transform_inverse(
		Matrix<T, Dynamic, Dynamic> joints,
		std::vector<TransformM<T>> &relative_transforms, Matrix<T, Dynamic, Dynamic> theta)
	{

		//Matrix<T, Dynamic, Dynamic> theta = getTheta();
		//Get rest pose theta
		Matrix<T, Dynamic, Dynamic> rest_theta = Matrix<T, Dynamic, Dynamic>::Constant(24, 3, 0);

		//Compute rotation matrices for root joints
		Matrix<T, Dynamic, Dynamic> result = build_rotation_matrix<T>(theta, joints.row(0), 0);
		Matrix<T, Dynamic, Dynamic> result_rest_theta = build_rotation_matrix<T>(rest_theta, joints.row(0), 0);

		std::vector<TransformM<T>> transforms;
		transforms.emplace_back(result);

		std::vector<TransformM<T>> transforms_rest_theta;
		transforms_rest_theta.emplace_back(result_rest_theta);

		Matrix<T, Dynamic, Dynamic> parents = getParents();

		for (unsigned int i = 0; i < 23; i++)
		{
			//Gk(theta, J)
			result = transforms[parents(i)].t * build_rotation_matrix<T>(theta, joints.row(i + 1), i + 1);
			transforms.emplace_back(result);

			//Gk*(rest_theta, J)
			result_rest_theta = transforms_rest_theta[parents(i)].t * build_rotation_matrix<T>(rest_theta, joints.row(i + 1), i + 1);
			transforms_rest_theta.emplace_back(result_rest_theta);
		}

		for (unsigned int i = 0; i < transforms.size(); i++)
		{
			//Gk * (Gk*) ^ (-1)
			relative_transforms.emplace_back(transforms[i].t * transforms_rest_theta[i].t.inverse());
		}
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> skinning(
		Matrix<T, Dynamic, Dynamic> template_vertices,
		std::vector<TransformM<T>> relative_transforms)
	{

		Matrix<T, Dynamic, Dynamic> dehomog_skinnedVertices = Matrix<T, Dynamic, Dynamic>::Constant(6890, 3, 0);
		Matrix<T, Dynamic, Dynamic> skinnedVertices = Matrix<T, Dynamic, Dynamic>::Constant(6890, 4, 0);
		Matrix<T, Dynamic, Dynamic> weights = getWeights();

		//Homogenize template vertices
		Matrix<T, Dynamic, Dynamic> homog_vertices = Matrix<T, Dynamic, Dynamic>::Constant(6890, 4, 1);
		homog_vertices.block(0, 0, 6890, 3) = template_vertices;

		for (unsigned int i = 0; i < template_vertices.rows(); i++)
		{

			for (unsigned int k = 0; k < 24; k++)
			{

				skinnedVertices.row(i) += weights(i, k) * relative_transforms[k].t * homog_vertices.row(i).transpose();
			}
		}

		//Dehomogenize skinned vertices
		dehomog_skinnedVertices = skinnedVertices.block(0, 0, 6890, 3);

		return dehomog_skinnedVertices;
	}

	template <typename T>
	Matrix<T, Dynamic, Dynamic> forward(Matrix<T, Dynamic, Dynamic> beta, Matrix<T, Dynamic, Dynamic> theta, bool regress)
	{

		Matrix<T, Dynamic, Dynamic> reshaped_shaped_vertices(6890, 3),
			reshaped_posed_vertices(6890, 3);

		//Shape and pose blending
		SMPL::blending<T>(reshaped_shaped_vertices, reshaped_posed_vertices, beta, theta);

		//Regress reshaped joints
		Matrix<T, Dynamic, Dynamic> shaped_template_joints(24, 3);
		shaped_template_joints = regressJoints<T>(reshaped_shaped_vertices);

		//Set parents of joints
		Matrix<T, Dynamic, Dynamic> parents(23, 1);
		parents << 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21;
		setParents(parents);

		//Perform rigid transform
		std::vector<TransformM<T>> relative_transforms;
		rigid_transform_inverse<T>(shaped_template_joints, relative_transforms, theta);

		//Perform skinning
		Matrix<T, Dynamic, Dynamic> transformedVertices(6890, 3);
		transformedVertices = skinning<T>(reshaped_posed_vertices, relative_transforms);

		return (regress == 1) ? regressJoints(transformedVertices) : transformedVertices;
	}
};
