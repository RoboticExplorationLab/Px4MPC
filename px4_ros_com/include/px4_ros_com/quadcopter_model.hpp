#pragma once

#include "Eigen/Eigen"
#include <iostream>
#include "px4_ros_com/quaternion_math.hpp"

const int NumStates = 13; // [x, y, z, q0, q1, q2, q3, u, v, w, p, q, r]
const int NumInputs = 4;

const int NumErrorStates = 12;
const int NumErrorInputs = 4;

const double gravity = 9.81;
const double mass = 1.535;
const double l = 0.250;
const double kt = 1.0;
const double km = 0.0245;
const Eigen::Matrix3d MOI = (Eigen::Matrix3d() << 0.029125, 0, 0,
                                                0, 0.029125, 0,
                                                0, 0, 0.055225).finished(); // Moment of Inertia matrix

const double max_thrust = 7.0;
const double max_thrust_sim = 7.0;

class QuadcopterModel
{
public:
    void Dynamics(double *x_dot, const double *x, const double *u) const;
    void Jacobian(double *jac, const double *x, const double *u) const;

    // void Dynamics_RK4(double *x_next, const double *x, const double *u, double dt) const;
    void Jacobian_RK4(double *jac_discrete, const double *jac, double dt) const;

    void get_reduced_jacobian(double *jac_reduced, double *jac_discrete, double *reference_state);
private:
};