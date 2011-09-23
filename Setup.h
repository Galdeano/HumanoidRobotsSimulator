

#define NbRobots 1
#define Video 0 // enregistre les images
#define Suspendu 0 // 1 pour si le robot ne touche pas le sol
#define Visualisation 0 // enregistre les valeurs numeriques de plusieurs variables

#define Te 0.005 // période d'échantillonage en s et en double


//Robot?
#define Sherpa 1
#define Generic 0

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
