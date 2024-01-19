#include "px4_ros_com/quadcopter_model.hpp"

void QuadcopterModel::Dynamics(double *x_dot, const double *x, const double *u) const
{
    Eigen::Map<Eigen::VectorXd> x_dot_vec(x_dot, NumStates);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, NumStates);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, NumInputs);

    const Eigen::Vector3d r = x_vec.block<3, 1>(0, 0);
    const Eigen::Vector4d q = x_vec.block<4, 1>(3, 0);
    const Eigen::Vector3d v = x_vec.block<3, 1>(7, 0);
    const Eigen::Vector3d w = x_vec.block<3, 1>(10, 0);

    Eigen::Matrix<double, 3, 4> temp1;
    temp1 << 0, 0, 0, 0,
        0, 0, 0, 0,
        kt, kt, kt, kt;

    Eigen::Matrix<double, 3, 4> temp2;
    temp2 << -1 * l * kt, l * kt, l * kt, -1 * l * kt,
        l * kt, -1 * l * kt, l * kt, -1 * l * kt,
        km, km, -1 * km, -1 * km;

    x_dot_vec.block<3, 1>(0, 0) = quaternion_math::qtoQ(q) * v;
    x_dot_vec.block<4, 1>(3, 0) = 0.5 * quaternion_math::get_G(q) * w;
    x_dot_vec.block<3, 1>(7, 0) = quaternion_math::qtoQ(q).transpose() * Eigen::Vector3d(0, 0, gravity) - temp1 * u_vec / mass - quaternion_math::get_hat(w) * v;
    x_dot_vec.block<3, 1>(10, 0) = MOI.inverse() * (-quaternion_math::get_hat(w) * MOI * w + temp2 * u_vec);
}

void QuadcopterModel::Jacobian(double *jac, const double *x, const double *u) const
{
    Eigen::Map<Eigen::Matrix<double, NumStates, NumStates + NumInputs>> J(jac);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, NumStates);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, NumInputs);

    const Eigen::Vector3d r = x_vec.block<3, 1>(0, 0);
    const Eigen::Vector4d q = x_vec.block<4, 1>(3, 0);
    const Eigen::Vector3d v = x_vec.block<3, 1>(7, 0);
    const Eigen::Vector3d w = x_vec.block<3, 1>(10, 0);

    // Calculate A matrix df/dx
    Eigen::Matrix<double, NumStates, NumStates> A = Eigen::Matrix<double, NumStates, NumStates>::Zero();

    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d eye = Eigen::Matrix3d::Identity();
    T(0, 0) = 1.0;
    T.block<3, 3>(1, 1) = -1 * eye;

    Eigen::Matrix<double, 4, 3> H = Eigen::Matrix<double, 4, 3>::Zero();
    H.block<3, 3>(1, 0) = eye;

    // drdot/dq
    A.block<3, 4>(0, 3) = H.transpose() * (quaternion_math::get_L(quaternion_math::get_L(q) * H * v) * T +
                                           quaternion_math::get_R(quaternion_math::get_R(T * q) * H * v));
    // dqdot/dq
    A.block<4, 4>(3, 3) = 0.5 * quaternion_math::get_R(H * w);
    // dvdot/dq
    Eigen::Vector3d gvec = Eigen::Vector3d::Zero();
    gvec(2) = gravity;
    A.block<3, 4>(7, 3) = H.transpose() * (quaternion_math::get_L(quaternion_math::get_L(T * q) * H * gvec) +
                                           quaternion_math::get_R(quaternion_math::get_R(q) * H * gvec) * T);
    // drdot/dv
    A.block<3, 3>(0, 7) = quaternion_math::qtoQ(q);
    // dvdot/dv
    A.block<3, 3>(7, 7) = -1 * quaternion_math::get_hat(w);
    // dqdot/dw
    A.block<4, 3>(3, 10) = 0.5 * quaternion_math::get_L(q) * H;
    // dvdot/dw
    A.block<3, 3>(7, 10) = quaternion_math::get_hat(v);
    // dwdot/dw
    A.block<3, 3>(10, 10) = MOI.inverse() * (quaternion_math::get_hat(MOI * w) - quaternion_math::get_hat(w) * MOI);

    // Calculate B matrix df/du
    Eigen::Matrix<double, NumStates, NumInputs> B = Eigen::Matrix<double, NumStates, NumInputs>::Zero();
    // dvdot/du
    B.block<1, 4>(9, 0) = -1 * (1 / mass) * kt * Eigen::Matrix<double, 1, 4>::Ones();
    // dwdot/du
    Eigen::Matrix<double, 3, 4> temp;
    temp << -1 * l * kt, l * kt, l * kt, -1 * l * kt,
        l * kt, -1 * l * kt, l * kt, -1 * l * kt,
        km, km, -1 * km, -1 * km;
    B.block<3, 4>(10, 0) = MOI.inverse() * temp;

    // Get Jacobian
    J.block<NumStates, NumStates>(0, 0) = A;
    J.block<NumStates, NumInputs>(0, NumStates) = B;
}

// void QuadcopterModel::Dynamics_RK4(double *x_next, const double *x, const double *u, double dt) const
// {
// // ......................................................................................................................
// }

void QuadcopterModel::Jacobian_RK4(double *jac_discrete, const double *jac, double dt) const
{
    Eigen::Map<Eigen::Matrix<double, NumStates, NumStates + NumInputs>> J_discrete(jac_discrete);
    Eigen::Map<const Eigen::Matrix<double, NumStates, NumStates + NumInputs>> J(jac);

    Eigen::Matrix<double, NumStates, NumStates> A = J.block<NumStates, NumStates>(0, 0);
    Eigen::Matrix<double, NumStates, NumInputs> B = J.block<NumStates, NumInputs>(0, NumStates);

    // Discretize by runge-kutta method 4th order
    Eigen::Matrix<double, NumStates, NumStates> eye13 = Eigen::Matrix<double, NumStates, NumStates>::Identity();

    Eigen::Matrix<double, NumStates, NumStates> f2 = (eye13 + 0.5 * dt * A) * A;
    Eigen::Matrix<double, NumStates, NumStates> f3 = (eye13 + 0.5 * dt * f2) * A;
    Eigen::Matrix<double, NumStates, 13> f4 = (eye13 + dt * f3) * A;
    Eigen::Matrix<double, NumStates, NumStates> Ad = eye13 + (dt / 6) * (A + 2 * f2 + 2 * f3 + f4);

    Eigen::Matrix<double, NumStates, NumInputs> Bd = B * dt;

    J_discrete.block<NumStates, NumStates>(0, 0) = Ad;
    J_discrete.block<NumStates, NumInputs>(0, NumStates) = Bd;
}

void QuadcopterModel::get_reduced_jacobian(double *jac_reduced, double *jac_discrete, double *reference_state)
{
    Eigen::Map<Eigen::Matrix<double, NumErrorStates, NumErrorStates + NumErrorInputs>> J_reduced(jac_reduced);
    Eigen::Map<Eigen::Matrix<double, NumStates, NumStates + NumInputs>> J_discrete(jac_discrete);   
    Eigen::Map<Eigen::VectorXd> x_vec(reference_state, NumStates);

    const Eigen::Vector4d q = x_vec.block<4, 1>(3, 0); 

    Eigen::Matrix<double, NumStates, NumStates> A = J_discrete.block<NumStates, NumStates>(0, 0);
    Eigen::Matrix<double, NumStates, NumInputs> B = J_discrete.block<NumStates, NumInputs>(0, NumStates);

    // Get reduced Jacobian
    J_reduced.block<NumErrorStates, NumErrorStates>(0, 0) = quaternion_math::get_E(q).transpose() * A * quaternion_math::get_E(q);
    J_reduced.block<NumErrorStates, NumErrorInputs>(0, NumErrorStates) = quaternion_math::get_E(q).transpose() * B;   

}