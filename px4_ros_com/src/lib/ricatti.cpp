#include "px4_ros_com/ricatti.hpp"

namespace ricatti
{
    void get_K(double *K, const double *jac_reduced, const double *Q, const double *S, const double *R)
    {
        Eigen::Map<Eigen::Matrix<double, 4, 12>> K_mat(K);
        Eigen::Map<const Eigen::Matrix<double, 12, 16>> jac_reduced_mat(jac_reduced);
        Eigen::Matrix<double, 12, 12> Q_mat = Eigen::Matrix<double, 12, 12>::Zero();
        Eigen::Matrix<double, 12, 12> S_mat = Eigen::Matrix<double, 12, 12>::Zero();
        Eigen::Matrix<double, 4, 4> R_mat = Eigen::Matrix<double, 4, 4>::Zero();
        for (int i = 0; i < 12; i++)
        {
            Q_mat(i, i) = Q[i];
            S_mat(i, i) = S[i];
        }
        for (int i = 0; i < 4; i++)
        {
            R_mat(i, i) = R[i];
        }

        // get A and B
        Eigen::Matrix<double, 12, 12> A = jac_reduced_mat.block<12, 12>(0, 0);
        Eigen::Matrix<double, 12, 4> B = jac_reduced_mat.block<12, 4>(0, 12);

        Eigen::Matrix<double, 4, 12> Kold = Eigen::Matrix<double, 4, 12>::Zero();

        for (int i = 0; i < 10000; i++)
        {
            K_mat = (R_mat + B.transpose() * S_mat * B).colPivHouseholderQr().solve(B.transpose() * S_mat * A);
            S_mat = Q_mat + K_mat.transpose() * R_mat * K_mat + (A - B * K_mat).transpose() * S_mat * (A - B * K_mat);
            
            if ((K_mat - Kold).cwiseAbs().maxCoeff() < 1e-7)
            {
                // cwiseAbs() returns an expression of the coefficient-wise absolute value
                std::cout << "Converged in " << i << " iterations" << std::endl;
                break;
            }
            else
            
            Kold = K_mat;
        }
        std::cout << "K: " << std::endl
                    << K_mat << std::endl;
    }
}