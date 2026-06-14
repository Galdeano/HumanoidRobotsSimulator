#ifndef H_OneFootTrajectory
#define H_OneFootTrajectory

#include <Eigen/Dense>
#include "uLink.h"

void OneFootTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & u, double t);

#endif
