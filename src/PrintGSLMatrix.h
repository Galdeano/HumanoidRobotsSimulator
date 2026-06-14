#ifndef H_PrintGSLMatrix
#define H_PrintGSLMatrix

#include <Eigen/Dense>

void PrintGSLMatrix(const Eigen::MatrixXd & M);

void PrintGSLMatrixTranspose(const Eigen::MatrixXd & M);

void PrintGSLVector(const Eigen::VectorXd & V);

#endif
