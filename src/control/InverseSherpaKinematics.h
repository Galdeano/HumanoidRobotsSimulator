#ifndef H_InverseSherpaKinematics
#define H_InverseSherpaKinematics

#include <Eigen/Dense>

void InverseSherpaKinematics(Eigen::Ref<Eigen::VectorXd> q, const Eigen::Ref<const Eigen::Vector3d>& pos);
void InverseSherpaKinematics(double * q, const double * pos);

#endif
