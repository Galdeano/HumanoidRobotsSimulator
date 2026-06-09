
#ifndef H_FindMother
#define H_FindMother

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file FindMother.h
 *  \brief Recursive mother ID search
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void FindMother(SuLINK uLINK[],int j)
 * \brief Recursively find the ID ok the mother link
 *
 * Must be used as follow:
 * \code FindMother(uLINK,1); \endcode
 *
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param j ID of the link
 */
void FindMother(SuLINK uLINK[],State *Status,int j);


#endif
