#ifndef H_CalcCoM
#define H_CalcCoM

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file CalcCoM.h
 *  \brief Compute the Center of Mass position
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void CalcCoM(SuLINK uLINK[],gsl_vector * com)
 * \brief Compute the Center of Mass position
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param com Out: vector containing the CoM position as result
 */
void CalcCoM(SuLINK uLINK[],gsl_vector * com);

#endif
