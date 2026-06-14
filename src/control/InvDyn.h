#ifndef H_InvDyn
#define H_InvDyn

#include <Eigen/Dense>
#include "uLink.h"

void InvDyn(SuLINK uLINK[], State *Status, int j, Eigen::VectorXd & u);

#endif
