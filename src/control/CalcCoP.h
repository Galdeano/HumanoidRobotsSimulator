#ifndef H_CalcCoP
#define H_CalcCoP

#include "uLink.h"

/** \file CalcCoP.h
 *  \brief Compute recursively the Center of Pressure position
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn double CalcCoP( SuLINK uLINK[], Eigen::Vector3d & CoP, int j)
 * \brief Compute recursively the Center of Pressure position
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param CoP vector containing the CoP position as result
 * \param j ID of the link
 */
double CalcCoP(SuLINK uLINK[], Eigen::Vector3d & CoP, int j);

#endif
