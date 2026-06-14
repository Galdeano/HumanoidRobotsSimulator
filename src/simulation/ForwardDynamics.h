#ifndef H_ForwardDynamics
#define H_ForwardDynamics

#include "uLink.h"

/** \file ForwardDynamics.h
 *  \brief Compute the Forward dynamic of the robot
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void ForwardDynamics(SuLINK uLINK[],State *Status,double Dtime,long t)
 * \brief Compute the Forward dynamic of the robot
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param Status Structure wich describe the state of the robot
 * \param t Actual number of iteration since the begening of execution of simulation
 */
void ForwardDynamics(SuLINK uLINK[],State *Status,long t);


#endif
