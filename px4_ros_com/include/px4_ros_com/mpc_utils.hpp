#pragma once

#include "Eigen/Eigen"
#include <iostream>
#include "altro/solver/typedefs.hpp"

using ContinuousDynamicsFunction = std::function<void(double *, const double *, const double *)>;
using ContinuousDynamicsJacobian = std::function<void(double *, const double *, const double *)>;

altro::ExplicitDynamicsFunction ForwardEulerDynamics(int n, int m,
                                                     const ContinuousDynamicsFunction f);
altro::ExplicitDynamicsJacobian ForwardEulerJacobian(int n, int m,
                                                     const ContinuousDynamicsFunction f,
                                                     const ContinuousDynamicsJacobian df);
altro::ExplicitDynamicsFunction RK4Dynamics(int n, int m,
                                            const ContinuousDynamicsFunction f);
altro::ExplicitDynamicsJacobian RK4Jacobian(int n, int m,
                                            const ContinuousDynamicsFunction f,
                                            const ContinuousDynamicsJacobian df);
altro::ExplicitDynamicsFunction MidpointDynamics(int n, int m, ContinuousDynamicsFunction f);
altro::ExplicitDynamicsJacobian MidpointJacobian(int n, int m, ContinuousDynamicsFunction f,
                                                 ContinuousDynamicsJacobian df);
Eigen::MatrixXd Interpolate(Eigen::MatrixXd x_start, Eigen::MatrixXd x_target, double index);