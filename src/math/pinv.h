#ifndef H_pinv
#define H_pinv

#include <Eigen/Dense>

int pinv(Eigen::MatrixXd & invM, const Eigen::MatrixXd & M);

#endif
