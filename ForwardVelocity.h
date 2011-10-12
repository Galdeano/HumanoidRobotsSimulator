#ifndef H_ForwardVelocity
#define H_ForwardVelocity

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file ForwardVelocity.h
 *  \brief Recursive velocities computations of robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void ForwardVelocity(SuLINK uLINK[],int j)
 * \brief Recursively compute the linear velocities for robot
 *
 * Must be used as follow:
 * \code ForwardVelocity(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void ForwardVelocity(SuLINK uLINK[], int j);

#endif
