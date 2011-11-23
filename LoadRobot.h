#ifndef H_LoadRobot
#define H_LoadRobot

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/** \file LoadRobot.h
 *  \brief Load a robot model
 *  \author    David Galdeano
 *  \date      11/2011
 *
 * These files allow loading a robot model
 */



/**
 * \fn void SaveStateXML(SuLINK uLINK[],float t)
 * \brief Save the state of the robot in a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotXML(SuLINK uLINK[],State *Status,char* RobotFile);


#endif
