#ifndef H_Setup
#define H_Setup

#define NbRobots 1
#define Video 1 // enregistre les images
#define Suspendu 0 // 1 pour si le robot ne touche pas le sol
#define Visualisation 0 // enregistre les valeurs numeriques de plusieurs variables

#define Te 0.005 // période d'échantillonage en s et en double


//Choose Robot model
#define Sherpa 0
#define Generic 1

#if Sherpa && Generic
#error Both Sherpa and Generic robot model are specified
#error Check Sherpa and Generic defs
#endif

#if Sherpa
#define  NbLinks  14
#endif

#if Generic
#define  NbLinks  22
#define  RLEG  2
#define  LLEG  8
#define  SPINE  14
#define  RARM  16
#define  LARM  19
#endif


//Choose Command
#define PD 1
#define Dynamic 0

#if PD && Dynamic
#error Both PD and Dynamic command are specified
#error Check PD and Dynamic defs
#endif

#endif


//Debug
#define StaticCOM 0


