#ifndef H_Rodrigues
#define H_Rodrigues

#include <Eigen/Dense>

/** \file Rodrigues.h
 *  \brief Compute the rotation matrix from an axis of rotation and an angle of rotation
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn int Rodrigues(Eigen::Matrix3d & rot, const Eigen::Vector3d & w, double dt)
 * \brief Compute the rotation matrix from an axis of rotation and an angle of rotation
 *
 * \param rot matrix of rotation
 * \param w axis of rotation
 * \param dt angle of rotation around the axis
 */
int Rodrigues(Eigen::Matrix3d & rot, const Eigen::Vector3d & w, double dt);

#endif
