#ifndef H_Stabilizator
#define H_Stabilizator

#include <Eigen/Dense>
#include "uLink.h"

void Stabilizator(SuLINK uLINK[], State *Status, Eigen::VectorXd & stab, double t);

#endif
