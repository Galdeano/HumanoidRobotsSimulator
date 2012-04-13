#ifndef H_DrawGround
#define H_DrawGround

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file DrawGround.h
 *  \brief Draw ground
 *  \author    David Galdeano
 *  \date      03/2012
 *
 */


/**
 * \fn void DrawGround(float x,float y,float z,float dx,float dy,float dz,)
 * \brief Draw ground
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void DrawGround(float x,float y,float z,float dx,float dy,float dz);

#endif
