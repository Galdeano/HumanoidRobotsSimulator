#ifndef H_InverseKinematics
#define H_InverseKinematics

#include <Eigen/Dense>

void InverseKinematics(Eigen::Ref<Eigen::VectorXd> q, const Eigen::Ref<const Eigen::Vector3d>& pos);
void InverseKinematics(double * q, const double * pos);

#endif
