#ifndef H_Rodrigues
#define H_Rodrigues

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file Rodrigues.h
 *  \brief Compute the rotation matrix from an axis of rotation and an angle of rotation
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void Rodrigues(gsl_matrix * rot,gsl_vector * w,double dt)
 * \brief Compute the rotation matrix from an axis of rotation and an angle of rotation
 *
 * \param rot matrix of rotation
 * \param w axis of rotation
 * \param dt angle of rotation around the axis
 */
void Rodrigues(gsl_matrix * rot,gsl_vector * w,double dt);

#endif
