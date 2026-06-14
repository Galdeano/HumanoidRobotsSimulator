#ifndef H_StaticTrajectory
#define H_StaticTrajectory

#include <Eigen/Dense>
#include "uLink.h"

void PDStaticTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & qd, double d_time, double t);
void PStaticTrajectory(SuLINK uLINK[], State *Status, Eigen::VectorXd & u, double t);

#endif
