#ifndef H_NodeForwardKinematics
#define H_NodeForwardKinematics

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** \file NodeForwardKinematics.h
 *  \brief Recursive kinematics computations of robot from any link
 *  \author    David Galdeano
 *  \date      02/2012
 *
 */


/**
 * \fn void NodeForwardKinematics(SuLINK uLINK[],int j, int sender)
 * \brief Recursively compute the kinematics for robot from a base with q coordinates
 *
 * Must be used as follow:
 * \code NodeForwardKinematics(uLINK,base,0); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the current link
 * \param sender ID of the link wich call the function
 */
void NodeForwardKinematics(SuLINK uLINK[],int j, int sender);


#endif
