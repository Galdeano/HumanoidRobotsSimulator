#ifndef H_CalcCoMJacobian
#define H_CalcCoMJacobian

#include <Eigen/Dense>
#include "uLink.h"

void CalcCoMJacobian(SuLINK uLINK[], State *Status, Eigen::MatrixXd & J, int base);

#endif
