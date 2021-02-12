
/*
// solvers for Algebraic Riccati equation
// - Iteration (continuous)
// - Iteration (discrete)
// - Arimoto-Potter
//
// author: Horibe Takamasa
*/
#pragma once

#include <Dense> 
#include <iostream>
#include <time.h>
#include <vector>

/* Itereation method for continuous model */
bool solveRiccatiIterationC(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
    Eigen::MatrixXd& P, const double dt = 0.001,
    const double& tolerance = 1.E-5,
    const int iter_max = 100000);

/* Itereation method for discrete model */
bool solveRiccatiIterationD(const Eigen::MatrixXd& Ad,
    const Eigen::MatrixXd& Bd, const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R, Eigen::MatrixXd& P,
    const double& tolerance = 1.E-5,
    const int iter_max = 100000);