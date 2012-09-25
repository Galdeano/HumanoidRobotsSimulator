#ifndef H_Scenarios
#define H_Scenarios


/** \file CalcCoM.h
 *  \brief Compute the articular trajectories for a desired scenario
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */


/**
 * \fn void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution_y)
 * \brief Compute the articular trajectories for a desired scenario
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param com Out: vector containing the CoM position as result
 */

void Scenario_desired_trajectory(double *qd, double t, int *desired_support, double *distribution_y);



#endif
