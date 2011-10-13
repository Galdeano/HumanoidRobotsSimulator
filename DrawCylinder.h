#ifndef H_DrawCylinder
#define H_DrawCylinder

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** \file DrawCylinder.h
 *  \brief Draw cylinder faces and vectices
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void DrawCylinder(SuLINK uLINK[],int j)
 * \brief Draw cylinder faces and vectices
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */

void DrawCylinder(SuLINK uLINK[],int j);

#endif
