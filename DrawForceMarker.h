#ifndef H_DrawForceMarker
#define H_DrawForceMarker

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file DrawForceMarker.h
 *  \brief Draw a line in 3D
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void DrawForceMarker(gsl_vector * ori,gsl_vector * vec)
 * \brief Draw a line in 3D
 *
 *
 * \param ori vector containing the origin position
 * \param vec vector containing the magnitude
 */
void DrawForceMarker(gsl_vector * ori,gsl_vector * vec);


#endif
