#ifndef H_InverseSherpaKinematics
#define H_InverseSherpaKinematics

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file InverseSherpaKinematics.h
 *  \brief Inverse kinematics computations for Sherpa robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void InverseSherpaKinematics(gsl_vector * q,gsl_vector * pos)
 * \brief Compute the inverse kinematics for the legs of Sherpa robot
 *
 * \param q Vector of angular position of leg
 * \param pos Cartesian coordinate of foot
 */
void InverseSherpaKinematics(gsl_vector * q,gsl_vector * pos);


#endif
