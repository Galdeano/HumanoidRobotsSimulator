#ifndef NULLSPACE_H_INCLUDED
#define NULLSPACE_H_INCLUDED

#include <Eigen/Dense>

void nullspace(const Eigen::MatrixXd & A, Eigen::MatrixXd & Anull);

#endif // NULLSPACE_H_INCLUDED
