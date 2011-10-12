#ifndef H_Cross
#define H_Cross

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


/** \file Cross.h
 *  \brief Compute the cross product of two vectors
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn int Cross(gsl_vector * a, gsl_vector * b, int i)
 * \brief Compute the cross product of two vectors
 *
 * \param a first vector
 * \param b second vector
 * \param i must be 0 to erase the first vector by the product, else the second one is erased to contain the result
 */
int Cross(gsl_vector * a, gsl_vector * b, int i);

#endif
