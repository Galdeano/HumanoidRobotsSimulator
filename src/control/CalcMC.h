#ifndef H_CalcMC
#define H_CalcMC

#include "uLink.h"

/** \file CalcMC.h
 *  \brief Recursive computations of center of mass position
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn void CalcMC( SuLINK uLINK[], Eigen::Vector3d & com, int j)
 * \brief Recursive computations of center of mass position
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param com Out: vector containing the center of mass position
 * \param j ID of the link
 */
void CalcMC(SuLINK uLINK[], Eigen::Vector3d & com, int j);

#endif
