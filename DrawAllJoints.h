#ifndef H_DrawAllJoints
#define H_DrawAllJoints

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file DrawAllJoints.h
 *  \brief Draw the model of the robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void DrawAllJoints(SuLINK uLINK[],int j)
 * \brief Draw the model of the robot
 *
 * Must be used as follow:
 * \code DrawAllJoints(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void DrawAllJoints(SuLINK uLINK[],int j);


#endif
