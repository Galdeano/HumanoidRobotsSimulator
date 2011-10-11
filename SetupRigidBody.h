
#ifndef H_SetupRigidBody
#define H_SetupRigidBody

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file SetupRigidBody.h
 *  \brief Kinematics computations of robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void SetupRigidBody(SuLINK uLINK[],int j)
 * \brief Compute the kinematics for robot from q coordinates
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void SetupRigidBody(SuLINK uLINK[],int j);

#endif
