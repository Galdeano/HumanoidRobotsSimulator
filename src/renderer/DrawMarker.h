#ifndef H_DrawMarker
#define H_DrawMarker

#include <Eigen/Dense>

/** \file DrawMarker.h
 *  \brief Draw a cross in 3D
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn void DrawMarker(const Eigen::Vector3d & com)
 * \brief Draw a cross in 3D
 *
 * \param com position of the marker
 */
void DrawMarker(const Eigen::Vector3d & com);

#endif
