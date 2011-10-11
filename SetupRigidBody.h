
#ifndef H_SetupRigidBody
#define H_SetupRigidBody

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file SetupRigidBody.h
 *  \brief Create the polygon faces and vectices for graphical representation
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void SetupRigidBody(SuLINK uLINK[],int j)
 * \brief Create the polygon faces and vectices for graphical representation
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void SetupRigidBody(SuLINK uLINK[],int j);

#endif
