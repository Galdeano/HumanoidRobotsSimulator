#ifndef H_CalcJacobianModif
#define H_CalcJacobianModif

#include <Eigen/Dense>
#include "uLink.h"

void CalcJacobianModif(SuLINK uLINK[], Eigen::MatrixXd & J, const Eigen::VectorXd & idx);

#endif
