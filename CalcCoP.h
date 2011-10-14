#ifndef H_CalcCoP
#define H_CalcCoP

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>



/** \file CalcCoP.h
 *  \brief Compute recursively the Center of Pressure position
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn double CalcCoP( SuLINK uLINK[], gsl_vector * CoP, int j)
 * \brief Compute recursively the Center of Pressure position
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param CoP vector containing the CoP position as result
 * \param j ID of the link
 */
double CalcCoP( SuLINK uLINK[], gsl_vector * CoP, int j);


#endif
