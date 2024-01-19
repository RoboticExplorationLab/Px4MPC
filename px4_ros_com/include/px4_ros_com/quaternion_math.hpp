#pragma once

#include "Eigen/Eigen"
#include <iostream>
namespace quaternion_math
{
    Eigen::Vector3d get_qtorp(const Eigen::Vector4d &q);
    Eigen::Matrix4d get_L(const Eigen::Vector4d &q);
    Eigen::Matrix4d get_R(const Eigen::Vector4d &q);
    Eigen::Matrix3d get_hat(const Eigen::Vector3d &vec);
    Eigen::Matrix3d qtoQ(const Eigen::Vector4d &q);
    Eigen::Matrix<double, 4, 3> get_G(const Eigen::Vector4d &q);
    Eigen::Matrix<double, 13, 12> get_E(const Eigen::Vector4d &q);
}
