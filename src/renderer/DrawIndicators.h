#ifndef H_DrawIndicators
#define H_DrawIndicators

#include <Eigen/Dense>
#include "Hoap_calc_zmp.h"
#include "uLink.h"

/** \file DrawIndicators.h
 *  \brief Draw severals indicators
 *  \author    David Galdeano
 *  \date      10/2011
 *
 */

/**
 * \fn void DrawIndicators(SuLINK uLINK[],State *Status,Eigen::Vector3d & com,Eigen::Vector3d & CoP,int ground)
 * \brief Draw the Center of Mass position, the Center of Pressure position, the feet position and the external force applied on foot
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param Status Structure wich describe the state of the robot
 * \param com vector containing the CoM position
 * \param CoP vector containing the CoP position
 * \param ground 0: display the CoM or 1: display the PCoM
 */
void DrawIndicators(SuLINK uLINK[], State *Status, Eigen::Vector3d & com, Eigen::Vector3d & CoP, int ground);
void Hoap_calc_zmp_visu(SuLINK uLINK[], State *Status, zmp_calc* zmp);

#endif
