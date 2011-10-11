#ifndef H_Setup
#define H_Setup

/** \file Setup.h
 *  \brief Contain setup of simulations
 *  \author    David Galdeano
 *  \date      10/2011
 */

#define NbRobots 1 /**< Number of robot simulated */
#define Video 1 /**< Record images of the simulation */
#define Suspendu 0 /**< 1 if the robot is hanged and dont touch the ground */
#define Visualisation 0 /**< Record variables of the simulation */

#define Te 0.005 /**< Sampling period of control loop in second  */


//Choose Robot model
#define Sherpa 0 /**< Sherpa robot model is used */
#define Generic 1 /**< Generic robot model is used */

#if Sherpa && Generic
#error Both Sherpa and Generic robot model are specified
#error Check Sherpa and Generic defs
#endif

#if Sherpa
#define  NbLinks  14 /**< Number of link declared (Number od DoF+2) */
#endif

#if Generic
#define  NbLinks  22 /**< Number of link declared (Number od DoF+2) */
#define  RLEG  2 /**< ID of first Right Leg Link */
#define  LLEG  8 /**< ID of first Left Leg Link */
#define  SPINE  14 /**< ID of first Spine Link */
#define  RARM  16 /**< ID of first Right Arm Link */
#define  LARM  19 /**< ID of first Left ARM Link */
#endif


//Choose Command
#define PD 1 /**< Proportional derivative command is used is used */
#define Dynamic 0 /**< Dynamic command is used is used */

#if PD && Dynamic
#error Both PD and Dynamic command are specified
#error Check PD and Dynamic defs
#endif

#endif


//Debug
#define StaticCOM 0 /**< 1: do not compute the dynamic of the robot */


