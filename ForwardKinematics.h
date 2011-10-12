#ifndef H_ForwardKinematics
#define H_ForwardKinematics

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** \file ForwardKinematics.h
 *  \brief Recursive kinematics computations of robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void ForwardKinematics(SuLINK uLINK[],int j)
 * \brief Recursively compute the kinematics for robot from q coordinates
 *
 * Must be used as follow:
 * \code ForwardKinematics(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void ForwardKinematics(SuLINK uLINK[],int j);


#endif
