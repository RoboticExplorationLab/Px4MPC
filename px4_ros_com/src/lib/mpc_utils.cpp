#include "px4_ros_com/mpc_utils.hpp"

altro::ExplicitDynamicsFunction ForwardEulerDynamics(int n, int m,
                                                     const ContinuousDynamicsFunction f)
{
  auto fd = [n, m, f](double *xn, const double *x, const double *u, float h)
  {
    Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
    f(xn, x, u); // xn is actually x_dot here, fill xn
    xn_vec = x_vec + h * xn_vec;
  };
  return fd;
}

altro::ExplicitDynamicsJacobian ForwardEulerJacobian(int n, int m,
                                                     const ContinuousDynamicsFunction f,
                                                     const ContinuousDynamicsJacobian df)
{
  auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h)
  {
    Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

    static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

    df(J.data(), x, u); // fill J
    J.leftCols(n) = In + h * J.leftCols(n);
    J.rightCols(m) = h * J.rightCols(m);
  };
  return fd;
}

altro::ExplicitDynamicsFunction RK4Dynamics(int n, int m,
                                            const ContinuousDynamicsFunction f)
{
  auto fd = [n, m, f](double *xn, const double *x, const double *u, float h)
  {
    Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

    static Eigen::VectorXd f1(n);
    f(f1.data(), x, u); // fill f1

    static Eigen::VectorXd f2(n);
    static Eigen::VectorXd x2(n);
    x2 = x_vec + 0.5 * h * f1;
    f(f2.data(), x2.data(), u); // fill f2

    static Eigen::VectorXd f3(n);
    static Eigen::VectorXd x3(n);
    x3 = x_vec + 0.5 * h * f2;
    f(f3.data(), x3.data(), u); // fill f3

    static Eigen::VectorXd f4(n);
    static Eigen::VectorXd x4(n);
    x4 = x_vec + h * f3;
    f(f4.data(), x4.data(), u); // fill f4

    xn_vec = x_vec + (h / 6) * (f1 + 2 * f2 + 2 * f3 + f4);
  };
  return fd;
}

altro::ExplicitDynamicsJacobian RK4Jacobian(int n, int m,
                                            const ContinuousDynamicsFunction f,
                                            const ContinuousDynamicsJacobian df)
{
  auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h)
  {
    Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

    // Discretize by runge-kutta method 4th order
    static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);
    static Eigen::MatrixXd A(n, n);
    static Eigen::MatrixXd B(n, m);
    static Eigen::MatrixXd f2(n, n);
    static Eigen::MatrixXd f3(n, n);
    static Eigen::MatrixXd f4(n, n);

    // Evaluate the Jacobian
    df(J.data(), x, u);
    A = J.leftCols(n);
    B = J.rightCols(m);

    f2 = (In + 0.5 * h * A) * A;
    f3 = (In + 0.5 * h * f2) * A;
    f4 = (In + h * f3) * A;
    J.leftCols(n) = In + (h / 6) * (A + 2 * f2 + 2 * f3 + f4); // Apply the chain rule
    J.rightCols(m) = B * h;
  };
  return fd;
}

altro::ExplicitDynamicsFunction MidpointDynamics(int n, int m, ContinuousDynamicsFunction f)
{
  auto fd = [n, m, f](double *xn, const double *x, const double *u, float h)
  {
    static Eigen::VectorXd xm(n);
    Eigen::Map<Eigen::VectorXd> xn_vec(xn, n);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);
    f(xm.data(), x, u);
    xm *= h / 2;
    xm.noalias() += x_vec;
    f(xn, xm.data(), u);
    xn_vec = x_vec + h * xn_vec;
  };
  return fd;
}

altro::ExplicitDynamicsJacobian MidpointJacobian(int n, int m, ContinuousDynamicsFunction f,
                                                 ContinuousDynamicsJacobian df)
{
  auto fd = [n, m, f, df](double *jac, const double *x, const double *u, float h)
  {
    static Eigen::MatrixXd A(n, n);
    static Eigen::MatrixXd B(n, m);
    static Eigen::MatrixXd Am(n, n);
    static Eigen::MatrixXd Bm(n, m);
    static Eigen::VectorXd xm(n);
    static Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n, n);

    Eigen::Map<Eigen::MatrixXd> J(jac, n, n + m);
    Eigen::Map<const Eigen::VectorXd> x_vec(x, n);
    Eigen::Map<const Eigen::VectorXd> u_vec(u, n);

    // Evaluate the midpoint
    f(xm.data(), x, u);
    xm = x_vec + h / 2 * xm;

    // Evaluate the Jacobian
    df(J.data(), x, u);
    A = J.leftCols(n);
    B = J.rightCols(m);

    // Evaluate the Jacobian at the midpoint
    df(J.data(), xm.data(), u);
    Am = J.leftCols(n);
    Bm = J.rightCols(m);

    // Apply the chain rule
    J.leftCols(n) = In + h * Am * (In + h / 2 * A);
    J.rightCols(m) = h * (Am * h / 2 * B + Bm);
  };
  return fd;
}

Eigen::MatrixXd Interpolate(Eigen::MatrixXd x_start, Eigen::MatrixXd x_target, double index)
{
  Eigen::MatrixXd x = x_start + index * (x_target - x_start);
  return x;
}