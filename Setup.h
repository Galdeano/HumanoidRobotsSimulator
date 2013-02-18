#ifndef H_Setup
#define H_Setup

/** \file Setup.h
 *  \brief Contain setup of simulations
 *  \author    David Galdeano
 *  \date      10/2011
 */


//sherpa model
//correction enseignements
//marche hoap



#define play_csv 0
#define zmp_filtering 0
#define zmp_feedback 0
#define old_zmp 0
#define save_data_long 0
#define save_data_quick 1
#define save_data_quick_temp 0

#define file_motor 0
#define reseau 1

#define local 1
#define replay 0
#define Tasks 1
#define file_human 0
#define file_hoap 0

#define Light 0
#define shadow 1
#define colorsGL 0
#define materials 1
#define mathGL 0
#define GroundResolution 20

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


/*
sandra

doodle:
alejandro
divine
joris
helene
guillaume
nico

mass pizza


Saint-Guilhem-le-Desert
Le Guilhaume d'Orange ou La Taverne de l Escuelle
demander auto petraud
echarpe cachemire
jardin des sens

*/

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
#define Sherpa 0
/*! \def Sherpa
    \brief Sherpa robot model is used
*/
#define Generic 1
/*! \def Generic
    \brief Generic robot model is used
*/
#define Human 0
/*! \def Human
    \brief Human model is used
*/
#if (Sherpa + Generic + Human)!=1
#error Both Sherpa and Generic robot model are specified
#error Check Sherpa and Generic defs
#endif



//Choose Command
#define PD 0
/*! \def PD
    \brief Proportional derivative command is used
*/
#define Dynamic 0
/*! \def Dynamic
    \brief Dynamic command is used
*/
#define Task 1
/*! \def Task
    \brief Task control is used
*/

#if (PD+ Dynamic + Task)!=1
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





