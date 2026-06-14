#ifndef H_rot2omega
#define H_rot2omega

#include <Eigen/Dense>

int rot2omega(const Eigen::Matrix3d & R, Eigen::Vector3d & w);

#endif
