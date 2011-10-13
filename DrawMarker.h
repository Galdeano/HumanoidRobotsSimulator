#ifndef H_DrawMarker
#define H_DrawMarker

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file DrawMarker.h
 *  \brief Draw a cross in 3D
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void DrawMarker(gsl_vector * com)
 * \brief Draw a cross in 3D
 *
 *
 * \param com position of the marker
 */
void DrawMarker(gsl_vector * com);


#endif
