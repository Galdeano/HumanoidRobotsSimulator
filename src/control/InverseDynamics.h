#ifndef H_InverseDynamics
#define H_InverseDynamics

#include <Eigen/Dense>
#include "uLink.h"

void InverseDynamics(SuLINK uLINK[], State *Status, int j, Eigen::Vector3d & f, Eigen::Vector3d & t);

#endif
