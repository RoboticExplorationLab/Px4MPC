#include "px4_ros_com/quaternion_math.hpp"
// Function declerations //

namespace quaternion_math
{
	Eigen::Matrix3d get_hat(const Eigen::Vector3d &vec)
	{
		Eigen::Matrix3d hat = Eigen::Matrix3d::Zero();
		hat(0, 1) = -vec(2);
		hat(0, 2) = vec(1);
		hat(1, 0) = vec(2);
		hat(1, 2) = -vec(0);
		hat(2, 0) = -vec(1);
		hat(2, 1) = vec(0);
		return hat;
	}

	Eigen::Matrix4d get_L(const Eigen::Vector4d &q)
	{
		double s = q(0);
		Eigen::Vector3d v = q.tail<3>();
		Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();

		Eigen::Matrix4d L;
		L.block<3, 3>(1, 1) = s * eye + get_hat(v);
		L.block<4, 1>(0, 0) = q;
		L.block<1, 3>(0, 1) = -v;
		return L;
	}
	Eigen::Matrix4d get_R(const Eigen::Vector4d &q)
	{
		double s = q(0);
		Eigen::Vector3d v = q.tail<3>();
		Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();

		Eigen::Matrix4d R;
		R.block<3, 3>(1, 1) = s * eye - get_hat(v);
		R.block<4, 1>(0, 0) = q;
		R.block<1, 3>(0, 1) = -v;
		return R;
	}

	Eigen::Vector3d get_qtorp(const Eigen::Vector4d &q)
	{
		Eigen::Vector3d v = q.tail<3>() / q(0);
		return v;
	}

	Eigen::Matrix3d qtoQ(const Eigen::Vector4d &q)
	{
		Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
		Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
		T(0, 0) = 1.0;
		T.block<3, 3>(1, 1) = -1 * eye;

		Eigen::Matrix<double, 4, 3> H = Eigen::Matrix<double, 4, 3>::Zero();
		H.block<3, 3>(1, 0) = eye;

		Eigen::Matrix4d L = get_L(q);
		Eigen::Matrix3d Q = H.transpose() * T * L * T * L * H;

		return Q;
	}

	Eigen::Matrix<double, 4, 3> get_G(const Eigen::Vector4d &q)
	{
		Eigen::Matrix<double, 4, 3> H = Eigen::Matrix<double, 4, 3>::Zero();
		H.block<3, 3>(1, 0) = Eigen::Matrix3d::Identity();
		Eigen::Matrix<double, 4, 3> G = get_L(q) * H;
		return G;
	}

	Eigen::Matrix<double, 13, 12> get_E(const Eigen::Vector4d &q)
	{
		Eigen::Matrix<double, 13, 12> E = Eigen::Matrix<double, 13, 12>::Zero();

		E.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
		E.block<4, 3>(3, 3) = get_G(q);
		E.block<6, 6>(7, 6) = Eigen::Matrix<double, 6, 6>::Identity();
		return E;
	}
}