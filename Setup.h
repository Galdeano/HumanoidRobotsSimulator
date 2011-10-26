#ifndef H_Setup
#define H_Setup

/** \file Setup.h
 *  \brief Contain setup of simulations
 *  \author    David Galdeano
 *  \date      10/2011
 */


//Choose scenario
#define Sc1 1
/*! \def Sc1
    \brief Scenario 1: Squat task, variation of Hip carthesian position on z axis
*/
#define Sc2 0
/*! \def Sc2
    \brief Scenario 2: variation of Hip carthesian position on y axis
*/
#define Sc3 0
/*! \def Sc3
    \brief Scenario 3: variation of Ankle articular orientation around y axis
*/
#define Sc4 0
/*! \def Sc4
    \brief Scenario 4: variation of Arm articular orientation around y axis
*/
#define Sc5 0
/*! \def Sc5
    \brief Scenario 5: variation of Trunk articular orientation around y axis
*/
#define Sc6 0
/*! \def Sc6
    \brief Scenario 6: variation of Trunk and Arm articular orientation around y axis
*/
#if Sc1 && Sc2 && Sc3 && Sc4 && Sc5 && Sc6
#error Only ons scenario can be choose
#error Check Sc* defs
#endif





#define NbRobots 1
/*! \def NbRobots
    \brief Number of robot simulated
*/
#define Video 1
/*! \def Video
    \brief Record images of the simulation
*/
#define Suspendu 0
/*! \def Suspendu
    \brief 1 if the robot is hanged and dont touch the ground
*/
#define Visualisation 0
/*! \def Visualisation
    \brief Record variables of the simulation
*/


#define Te 0.005
/*! \def Te
    \brief Sampling period of control loop in second
*/

#define Dtime 0.0001
/*! \def Dtime
    \brief Simulation sampling period
*/

//Choose Robot model
#define Sherpa 0
/*! \def Sherpa
    \brief Sherpa robot model is used
*/
#define Generic 1
/*! \def Generic
    \brief Generic robot model is used
*/

#if Sherpa && Generic
#error Both Sherpa and Generic robot model are specified
#error Check Sherpa and Generic defs
#endif


#if Sherpa
#define  NbLinks  14
/*! \def NbLinks
    \brief Number of link declared (Number od DoF+2)
*/
#endif


#if Generic
#define  NbLinks  22
/*! \def NbLinks
    \brief Number of link declared (Number od DoF+2)
*/
#define  RLEG  2
/*! \def RLEG
    \brief ID of first Right Leg Link
*/
#define  LLEG  8
/*! \def LLEG
    \brief ID of first Left Leg Link
*/
#define  SPINE  14
/*! \def SPINE
    \brief ID of first Spine Link
*/
#define  RARM  16
/*! \def RARM
    \brief ID of first Right Arm Link
*/
#define  LARM  19
/*! \def LARM
    \brief ID of first Left Arm Link
*/
#endif


//Choose Command
#define PD 1
/*! \def PD
    \brief Proportional derivative command is used is used
*/
#define Dynamic 0
/*! \def Dynamic
    \brief Dynamic command is used is used
*/

#if PD && Dynamic
#error Both PD and Dynamic command are specified
#error Check PD and Dynamic defs
#endif

#endif


//Debug
#define StaticCOM 0
/*! \def StaticCOM
    \brief 1: do not compute the dynamic of the robot

    For debug use only
*/

