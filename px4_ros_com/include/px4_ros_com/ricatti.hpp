#pragma once

#include "Eigen/Eigen"
#include <iostream>

namespace ricatti
{
    void get_K(double *K, const double *jac_reduced, const double *Q, const double *S, const double *R) ;
}