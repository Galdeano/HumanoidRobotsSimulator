#ifndef H_LoadRobot_f
#define H_LoadRobot_f


#include "uLINK_f.h"

/** \file LoadRobot_f.h
 *  \brief Load a robot model
 *  \author    David Galdeano
 *  \date      11/2011
 *
 * These files allow loading a robot model
 */


/**
 * \fn void LoadRobotParserXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotParserXML_f(Struct_uLINK uLINK[], Struct_State *Status, char* RobotFile);

#endif
