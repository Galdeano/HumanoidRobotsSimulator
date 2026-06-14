#ifndef H_CalcVWerrOri
#define H_CalcVWerrOri

#include <Eigen/Dense>
#include "uLink.h"

void CalcVWerrOri(SuLINK uLINK[], Eigen::VectorXd & err, const Eigen::Vector3d & p, const Eigen::Matrix3d & R, const Eigen::VectorXd & idx);

#endif
