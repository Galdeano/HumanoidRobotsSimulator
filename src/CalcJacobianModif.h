#ifndef H_CalcJacobianModif
#define H_CalcJacobianModif

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

///** \file CalcJacobianModif.h
// *  \brief Recursive computations of center of mass position
// *  \author    David Galdeano
// *  \date      10/2011
// *
// */
//
//
///**
// * \fn void CalcMC( SuLINK uLINK[], gsl_vector * com, int j)
// * \brief Recursive computations of center of mass position
// *
// * Must be used as follow:
// * \code CalcMC(uLINK,com,1); \endcode
// *
// *
// * \param uLINK[] Structure wich describe the robot link by link
// * \param com Out: vector containing the center of mass position
// * \param j ID of the link
// */
void CalcJacobianModif( SuLINK uLINK[], gsl_matrix * J, gsl_vector * idx);

#endif
