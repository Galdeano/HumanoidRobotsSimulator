#ifndef H_LoadRobot
#define H_LoadRobot

#include "uLink.h"

/** \file LoadRobot.h
 *  \brief Load a robot model
 *  \author    David Galdeano
 *  \date      11/2011
 *
 * These files allow loading a robot model
 */

/**
 * \fn void LoadRobotXML(SuLINK uLINK[],State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotXML(SuLINK uLINK[], State *Status, char* RobotFile);

/**
 * \fn void LoadRobotParserXML(SuLINK uLINK[],State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotParserXML(SuLINK uLINK[], State *Status, char* RobotFile);

#endif
