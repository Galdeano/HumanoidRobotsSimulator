#ifndef H_Cross
#define H_Cross

#include <Eigen/Dense>

/** \file Cross.h
 *  \brief Compute the cross product of two vectors
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn void Cross(Eigen::Vector3d & a, Eigen::Vector3d & b, int i)
 * \brief Compute the cross product of two vectors
 *
 * \param a first vector
 * \param b second vector
 * \param i must be 0 to erase the first vector by the product, else the second one is erased to contain the result
 */
void Cross(Eigen::Vector3d & a, Eigen::Vector3d & b, int i);

#endif
