#ifndef H_DrawForceMarker
#define H_DrawForceMarker

#include <Eigen/Dense>

/** \file DrawForceMarker.h
 *  \brief Draw a line in 3D
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn void DrawForceMarker(const Eigen::Vector3d & ori, const Eigen::Vector3d & vec)
 * \brief Draw a line in 3D
 *
 * \param ori vector containing the origin position
 * \param vec vector containing the magnitude
 */
void DrawForceMarker(const Eigen::Vector3d & ori, const Eigen::Vector3d & vec);

#endif
