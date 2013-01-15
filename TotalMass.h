#ifndef H_TotalMass
#define H_TotalMass

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLink.h"

/** \file TotalMass.h
 *  \brief Recursive computations of robot mass
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void TotalMass(SuLINK uLINK[],int j)
 * \brief Recursive computations of robot mass
 *
 * Must be used as follow:
 * \code TotalMass(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
double TotalMass(SuLINK uLINK[],int j);

#endif
