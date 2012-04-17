#ifndef H_LoadRobot
#define H_LoadRobot

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLink.h"
#include "uLINK_f.h"

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
 * \fn void LoadRobotXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotXML_f(Struct_uLINK uLINK[], Struct_State *Status, char* RobotFile);


/**
 * \fn void LoadRobotParserXML(SuLINK uLINK[],State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotParserXML(SuLINK uLINK[], State *Status, char* RobotFile);


/**
 * \fn void LoadRobotParserXML_f(Struct_uLINK uLINK[],Struct_State *Status,char* RobotFile)
 * \brief Load the robot from a XML file
 *
 * \param uLINK[] Structure wich describe the robot link by link
 * \param t Current time
 */
void LoadRobotParserXML_f(Struct_uLINK uLINK[], Struct_State *Status, char* RobotFile);

#endif
