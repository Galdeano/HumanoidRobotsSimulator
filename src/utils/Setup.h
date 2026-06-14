#ifndef H_Setup
#define H_Setup

/** \file Setup.h
 *  \brief Contain setup of simulations
 *  \author    David Galdeano
 *  \date      10/2011
 */


#define play_csv 0
#define zmp_filtering 1
#define zmp_feedback 1//
#define old_zmp 0
#define save_data_long 0
#define save_data_quick 1
#define save_data_quick_temp 1

#define file_motor 0
#define network 0

#define local 1//
#define replay 0
#define Tasks 1
#define file_human 0
#define file_hoap 0
#define file_walk 1//

#define Light 0//
#define shadow 1
#define colorsGL 0
#define materials 1
#define mathGL 0
#define GroundResolution 20

#define oritrunk 1//
#define oritrunk3 1//

#define LoadObj 1
/*! \def LoadObj
    \brief 1: Load and displays the mechs associated to the model if they exists
*/


#define Scenarios 0
/*! \def Scenarios
    \brief Uses trajectories from scenarios for demonstrations purposes
*/
#define Trajectories 1
/*! \def Trajectories
    \brief Uses trajectories for tasks
*/
#define Ext_traj 0
/*! \def Ext_traj
    \brief Uses trajectories from file
*/
#if (Scenarios + Trajectories + Ext_traj)!=1
#error Choose between Typical scenarios or Task trajectories
#error Check Sc* defs
#endif




#if Scenarios
//Choose scenario
#define Sc1 0
/*! \def Sc1
    \brief Scenario 1: Squat task, variation of Hip Cartesian position on z axis
*/
#define Sc2 0
/*! \def Sc2
    \brief Scenario 2: Variation of Hip carthesian position on y axis
*/
#define Sc3 0
/*! \def Sc3
    \brief Scenario 3: Variation of Ankle articular orientation around y axis
*/
#define Sc4 1
/*! \def Sc4
    \brief Scenario 4: Variation of Arm articular orientation around y axis
*/
#define Sc5 0
/*! \def Sc5
    \brief Scenario 5: Variation of Trunk articular orientation around y axis
*/
#define Sc6 0
/*! \def Sc6
    \brief Scenario 6: Variation of Trunk and Arm articular orientation around y axis
*/
#define Sc7 0
/*! \def Sc7
    \brief Scenario 7: Variation of Hip and Ankle articular orientation around x axis
*/

#if (Sc1 + Sc2 + Sc3 + Sc4 + Sc5 + Sc6+ Sc7)!=1
#error Only one scenario can be choose
#error Check Sc* defs
#endif
#endif




#define Video 0
/*! \def Video
    \brief Record images of the simulation
*/
#define Suspendu 0
/*! \def Suspendu
    \brief 1 if the robot is hanged and dont touch the ground
*/
#define Visualisation 1
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
#define ROBOT_SHERPA 0
/*! \def ROBOT_SHERPA
    \brief Sherpa robot model is used
*/
#define ROBOT_GENERIC 1
/*! \def ROBOT_GENERIC
    \brief Generic robot model is used
*/
#define ROBOT_HUMAN 0
/*! \def ROBOT_HUMAN
    \brief Human model is used
*/
#if (ROBOT_SHERPA + ROBOT_GENERIC + ROBOT_HUMAN)!=1
#error Both Sherpa and Generic robot model are specified
#error Check Sherpa and Generic defs
#endif



//Choose Command
#define CMD_PD 0
/*! \def CMD_PD
    \brief Proportional derivative command is used
*/
#define CMD_DYNAMIC 0
/*! \def CMD_DYNAMIC
    \brief Dynamic command is used
*/
#define CMD_TASK 1
/*! \def CMD_TASK
    \brief Task control is used
*/

#if (CMD_PD + CMD_DYNAMIC + CMD_TASK)!=1
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


#define VisuArticularsLimits 0
/*! \def VisuArticulaLimits
    \brief 1: display articular range of motion
*/

#define VisuTorquesColor 1
/*! \def VisuTorquesColor
    \brief 1: display toques via color change on joints articulation
*/

#define baseFoot 13
/*! \def base
    \brief base link for computation
*/


// Contacts
#define xContact 5
#define yContact 4
#define linearDamper 1
/*! \def linearDamper
    \brief 1: Use a spring damper contact
*/

#define nonLinearDamper 0
/*! \def nonLinearDamper
    \brief 1: Use a non linear damper contact
*/

#if (linearDamper + nonLinearDamper)!=1
#error Both linearDamper and nonLinearDamper contact model are specified
#error Check linearDamper and nonLinearDamper defs
#endif





