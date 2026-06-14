#include <string.h>
#include <Eigen/Dense>



#include "uLink.h"
#include "SetupRigidBody.h"
#include "FindMother.h"
#include "ForwardKinematics.h"
#include "ForwardVelocity.h"
#include "CalcCoM.h"

#include "Setup.h"

#if ROBOT_SHERPA
void SetupRobot(SuLINK uLINK[],State *Status)
{


    double Hy=0.155*2;
    double Hx=0.0468;
    double Lc=0.5073;
    double Lt=0.510;
    double Lp=0.1104;

    // Inertia elements of the Sherpa robot - 20091021SK
    double mPelvis=3.2+1.9+1; // [kg]
    double mUpperLimb=5+3;
    double mLowerLimb=9.9+1;
    double mFoot=0.8+0.8;
    //Total=mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot
    //printf("%8.4f \n",mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot);


    int i;
    for(i=1; i<NbLinks; i++)
    {
        uLINK[i].q = 0.0;
        uLINK[i].dq = 0.0;
        uLINK[i].ddq = 0.0;
        uLINK[i].Ir = 0.0;
        uLINK[i].gr = 0.0;
        uLINK[i].u = 0.0;
        uLINK[i].ug = 0.0;
        uLINK[i].uef = 0.0;
        uLINK[i].u_joint = 0.0;
        uLINK[i].isPolygon=0;
                                                                                                
        
    }


    Status->dof=12+6;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5;
    Status->right_foot_ID=7;
    Status->left_foot_ID=13;
    Status->integral_R=0;
    Status->integral_L=0;
                            


    //Robot Configuration
    strcpy(uLINK[1].name, "BODY");
    uLINK[1].m  = 0 ;
    uLINK[1].color  = 3 ;
    uLINK[1].sister  = 0 ;//0
    uLINK[1].child  = 2 ;
    uLINK[1].a(2) = 1;
    uLINK[1].p(2) = Lc+Lt+Lp;

    uLINK[1].R(0, 0) = 1;
    uLINK[1].R(1, 1) = 1;
    uLINK[1].R(2, 2) = 1;

    uLINK[1].c(0) = -0.00526;
    uLINK[1].c(2) = 0.09054;
    uLINK[1].I(0, 0) = 0.06714208509;
    uLINK[1].I(0, 1) = 0.00000021658;
    uLINK[1].I(0, 2) = -0.00057445503;
    uLINK[1].I(1, 0) = 0.00000021658;
    uLINK[1].I(1, 1) = 0.01982723311;
    uLINK[1].I(1, 2) = 0.00000091180;
    uLINK[1].I(2, 0) = -0.00057445503;
    uLINK[1].I(2, 1) = 0.00000091180;
    uLINK[1].I(2, 2) = 0.05554470420;



//uLINK(BODY).c = [-5.26, 0, 90.54]'*10^(-3);
//uLINK(BODY).I = [67142085.09,216.58,-574455.03;216.58,19827233.11,911.80;-574455.03,911.80,55544704.20]*10^(-9);



    strcpy(uLINK[2].name, "RLEG_J0");
    uLINK[2].m  = mPelvis/2 ;
    uLINK[2].color  = 2 ;
    uLINK[2].sister  = 8 ;//8
    uLINK[2].child  = 3 ;
    uLINK[2].c(0) = -0.00526;
    uLINK[2].c(2) = 0.09054;
    uLINK[2].b(0) = Hx;
    uLINK[2].b(1) = -Hy/2;
    uLINK[2].a(0) = 1;
    uLINK[2].I(0, 0) = 0.06714208509;
    uLINK[2].I(0, 1) = 0.00000021658;
    uLINK[2].I(0, 2) = -0.00057445503;
    uLINK[2].I(1, 0) = 0.00000021658;
    uLINK[2].I(1, 1) = 0.01982723311;
    uLINK[2].I(1, 2) = 0.00000091180;
    uLINK[2].I(2, 0) = -0.00057445503;
    uLINK[2].I(2, 1) = 0.00000091180;
    uLINK[2].I(2, 2) = 0.05554470420;


    strcpy(uLINK[3].name, "RLEG_J1");
    uLINK[3].m  = 0 ;
    uLINK[3].color  = 2 ;
    uLINK[3].sister  = 0 ;
    uLINK[3].child  = 4 ;
    uLINK[3].a(1) = 1;
    uLINK[3].c(0) = 0.0068;
    uLINK[3].c(1) = -0.0025;
    uLINK[3].c(2) = 0.00076;
    uLINK[3].I(0, 0) = 0.00004133363;
    uLINK[3].I(0, 1) = -0.00000225009;
    uLINK[3].I(0, 2) = -0.00000032224;
    uLINK[3].I(1, 0) = -0.00000225009;
    uLINK[3].I(1, 1) = 0.00003404560;
    uLINK[3].I(1, 2) = 0.00000025384;
    uLINK[3].I(2, 0) = -0.00000032224;
    uLINK[3].I(2, 1) = 0.00000025384;
    uLINK[3].I(2, 2) = 0.00006366584;

    strcpy(uLINK[4].name, "RLEG_J2");
    uLINK[4].m  = mUpperLimb ;
    uLINK[4].color  = 2 ;
    uLINK[4].sister  = 0 ;
    uLINK[4].child  = 5 ;
    uLINK[4].a(2) = 1;
    uLINK[4].c(0) = 0.032;
    uLINK[4].c(2) = -0.204;
    uLINK[4].I(0, 0) = 0.08625887792;
    uLINK[4].I(0, 1) = -0.00009806736;
    uLINK[4].I(0, 2) = 0.00087486898;
    uLINK[4].I(1, 0) = -0.00009806736;
    uLINK[4].I(1, 1) = 0.08320117734;
    uLINK[4].I(1, 2) = 0.00054829757;
    uLINK[4].I(2, 0) = 0.00087486898;
    uLINK[4].I(2, 1) = 0.00054829757;
    uLINK[4].I(2, 2) = 0.01211293791;

    strcpy(uLINK[5].name, "RLEG_J3");
    uLINK[5].m  = mLowerLimb ;
    uLINK[5].color  = 0 ;
    uLINK[5].sister  = 0 ;
    uLINK[5].child  = 6 ;
    uLINK[5].a(1) = 1;
    uLINK[5].b(2) = -Lc;
    uLINK[5].c(2) = -0.255;
    uLINK[5].I(0, 0) = 0.19372258067;
    uLINK[5].I(0, 1) = 0.00000963489;
    uLINK[5].I(0, 2) = 0.02878549993;
    uLINK[5].I(1, 0) = 0.00000963489;
    uLINK[5].I(1, 1) = 0.19983382882;
    uLINK[5].I(1, 2) = -0.00125880365;
    uLINK[5].I(2, 0) = 0.02878549993;
    uLINK[5].I(2, 1) = -0.00125880365;
    uLINK[5].I(2, 2) = 0.03540246245;

    strcpy(uLINK[6].name, "RLEG_J4");
    uLINK[6].m  = 0 ;
    uLINK[6].color  = 1 ;
    uLINK[6].sister  = 0 ;
    uLINK[6].child  = 7 ;
    uLINK[6].a(1) = 1;
    uLINK[6].b(2) = -Lt;
    //uLINK[6].c(2) = -0.255;

    strcpy(uLINK[7].name, "RLEG_J5");
    uLINK[7].m  = mFoot ;
    uLINK[7].color  = 1 ;
    uLINK[7].sister  = 0 ;
    uLINK[7].child  = 0 ;
    uLINK[7].a(0) = 1;

    uLINK[7].c(0) = 0.0859;
    uLINK[7].c(2) = -0.0671;
    uLINK[7].I(0, 0) = 0.00354735101;
    uLINK[7].I(0, 1) = -0.00001373757;
    uLINK[7].I(0, 2) = -0.00135282832;
    uLINK[7].I(1, 0) = -0.00001373757;
    uLINK[7].I(1, 1) = 0.00736336832;
    uLINK[7].I(1, 2) = 0.00000176965;
    uLINK[7].I(2, 0) = -0.00135282832;
    uLINK[7].I(2, 1) = 0.00000176965;
    uLINK[7].I(2, 2) = 0.00555234777;




    strcpy(uLINK[8].name, "LLEG_J0");
    uLINK[8].m  = mPelvis/2 ;
    uLINK[8].color  = 2 ;
    uLINK[8].sister  = 0 ;
    uLINK[8].child  = 9 ;
    uLINK[8].c(0) = -0.00526;
    uLINK[8].c(2) = 0.09054;
    uLINK[8].a(0) = 1;
    uLINK[8].b(0) = Hx;
    uLINK[8].b(1) = Hy/2;
    uLINK[8].I(0, 0) = 0.06714208509;
    uLINK[8].I(0, 1) = 0.00000021658;
    uLINK[8].I(0, 2) = -0.00057445503;
    uLINK[8].I(1, 0) = 0.00000021658;
    uLINK[8].I(1, 1) = 0.01982723311;
    uLINK[8].I(1, 2) = 0.00000091180;
    uLINK[8].I(2, 0) = -0.00057445503;
    uLINK[8].I(2, 1) = 0.00000091180;
    uLINK[8].I(2, 2) = 0.05554470420;

    strcpy(uLINK[9].name, "LLEG_J1");
    uLINK[9].m  = 0 ;
    uLINK[9].color  = 2 ;
    uLINK[9].sister  = 0 ;
    uLINK[9].child  = 10 ;
    uLINK[9].a(1) = 1;
    uLINK[9].c(0) = 0.0068;
    uLINK[9].c(1) = 0.0025;
    uLINK[9].c(2) = 0.00076;
    uLINK[9].I(0, 0) = 0.00004133363;
    uLINK[9].I(0, 1) = -0.00000225009;
    uLINK[9].I(0, 2) = -0.00000032224;
    uLINK[9].I(1, 0) = -0.00000225009;
    uLINK[9].I(1, 1) = 0.00003404560;
    uLINK[9].I(1, 2) = 0.00000025384;
    uLINK[9].I(2, 0) = -0.00000032224;
    uLINK[9].I(2, 1) = 0.00000025384;
    uLINK[9].I(2, 2) = 0.00006366584;

    strcpy(uLINK[10].name, "LLEG_J2");
    uLINK[10].m  = mUpperLimb ;
    uLINK[10].color  = 2 ;
    uLINK[10].sister  = 0 ;
    uLINK[10].child  = 11 ;
    uLINK[10].a(2) = 1;
    uLINK[10].c(0) = 0.032;
    uLINK[10].c(2) = -0.204;
    uLINK[10].I(0, 0) = 0.08625887792;
    uLINK[10].I(0, 1) = -0.00009806736;
    uLINK[10].I(0, 2) = 0.00087486898;
    uLINK[10].I(1, 0) = -0.00009806736;
    uLINK[10].I(1, 1) = 0.08320117734;
    uLINK[10].I(1, 2) = 0.00054829757;
    uLINK[10].I(2, 0) = 0.00087486898;
    uLINK[10].I(2, 1) = 0.00054829757;
    uLINK[10].I(2, 2) = 0.01211293791;

    strcpy(uLINK[11].name, "LLEG_J3");
    uLINK[11].m  = mLowerLimb ;
    uLINK[11].color  = 0 ;
    uLINK[11].sister  = 0 ;
    uLINK[11].child  = 12 ;
    uLINK[11].a(1) = 1;
    uLINK[11].b(2) = -Lc;
    uLINK[11].c(2) = -0.255;
    uLINK[11].I(0, 0) = 0.19372258067;
    uLINK[11].I(0, 1) = 0.00000963489;
    uLINK[11].I(0, 2) = 0.02878549993;
    uLINK[11].I(1, 0) = 0.00000963489;
    uLINK[11].I(1, 1) = 0.19983382882;
    uLINK[11].I(1, 2) = -0.00125880365;
    uLINK[11].I(2, 0) = 0.02878549993;
    uLINK[11].I(2, 1) = -0.00125880365;
    uLINK[11].I(2, 2) = 0.03540246245;

    strcpy(uLINK[12].name, "LLEG_J4");
    uLINK[12].m  = 0 ;
    uLINK[12].color  = 1 ;
    uLINK[12].sister  = 0 ;
    uLINK[12].child  = 13 ;
    uLINK[12].a(1) = 1;
    uLINK[12].b(2) = -Lt;
    //uLINK[12].c(2) = -0.255;

    strcpy(uLINK[13].name, "LLEG_J5");
    uLINK[13].m  = mFoot ;
    uLINK[13].color  = 1 ;
    uLINK[13].sister  = 0 ;
    uLINK[13].child  = 0 ;
    uLINK[13].a(0) = 1;
    uLINK[13].c(0) = 0.0859;
    uLINK[13].c(2) = -0.0671;
    uLINK[13].I(0, 0) = 0.00354735101;
    uLINK[13].I(0, 1) = -0.00001373757;
    uLINK[13].I(0, 2) = -0.00135282832;
    uLINK[13].I(1, 0) = -0.00001373757;
    uLINK[13].I(1, 1) = 0.00736336832;
    uLINK[13].I(1, 2) = 0.00000176965;
    uLINK[13].I(2, 0) = -0.00135282832;
    uLINK[13].I(2, 1) = 0.00000176965;
    uLINK[13].I(2, 2) = 0.00555234777;


    // Contacts
    uLINK[1].shape[0]=0.1;
    uLINK[1].shape[1]=0.3;
    uLINK[1].shape[2]=0.04;
    uLINK[1].com[0]=0;
    uLINK[1].com[1]=0;
    uLINK[1].com[2]=0.05;
    SetupRigidBody(uLINK,1);
    uLINK[1].supportHeight=0.0;

    uLINK[7].shape[0]=0.28;
    uLINK[7].shape[1]=0.08;
    uLINK[7].shape[2]=0.04;
    uLINK[7].com[0]=0.0853;
    uLINK[7].com[1]=0;
    uLINK[7].com[2]=-0.09;
    SetupRigidBody(uLINK,7);
    uLINK[7].supportHeight=0.0;

    uLINK[13].shape[0]=0.28;
    uLINK[13].shape[1]=0.08;
    uLINK[13].shape[2]=0.04;
    uLINK[13].com[0]=0.0853;
    uLINK[13].com[1]=0;
    uLINK[13].com[2]=-0.09;
    SetupRigidBody(uLINK,13);
    uLINK[13].supportHeight=0.0;

    //Initial Position
//    uLINK[3].q  = -0.1 ;
//    uLINK[5].q  = 0.2 ;
//    uLINK[6].q  = -0.1 ;
//    uLINK[9].q  = -0.1 ;
//    uLINK[11].q  = 0.2 ;
//    uLINK[12].q  = -0.1 ;



//    double ToRad = M_PI/180;
//
//    uLINK[3].q  = -10*ToRad ;
//    uLINK[5].q  = 20*ToRad ;
//    uLINK[6].q  = -10*ToRad ;
//    uLINK[9].q  = -10*ToRad ;
//    uLINK[11].q  = 20*ToRad ;
//    uLINK[12].q  = -10*ToRad ;



    FindMother(uLINK,1);

    ForwardKinematics(uLINK,1);

    ForwardVelocity(uLINK,1);

    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    CalcCoM(uLINK,com);
    Status->com_old = com;
    


//    FILE * f = fopen ("test.dat", "w");
//    for(i=1; i<14; i++)
//    {
//        gsl_vector_fprintf (f, uLINK[i].p, "%.5g");
//    }
//    fclose (f);


}
#endif

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


#if ROBOT_GENERIC
void SetupRobot(SuLINK uLINK[],State *Status)
{


    double Hy=0.155*2;
    double Hx=0.0468;
    double Lc=0.5073;
    double Lt=0.510;
    double Lp=0.1104;

    int  RLEG = 2;
    int  LLEG = 8;
    int  SPINE = 14;
    int  RARM = 16;
    int  LARM = 19;

    int  NbLinks = 22;

    // Inertia elements of the Sherpa robot - 20091021SK
    double mPelvis=3.2+1.9+1; // [kg]
    double mUpperLimb=5+3;
    double mLowerLimb=9.9+1;
    double mFoot=0.8+0.8;
    //Total=mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot
    //printf("%8.4f \n",mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot);


    int i;
    for(i=1; i<NbLinks; i++)
    {
        uLINK[i].q = 0.0;
        uLINK[i].dq = 0.0;
        uLINK[i].ddq = 0.0;
        uLINK[i].Ir = 0.0;
        uLINK[i].gr = 0.0;
        uLINK[i].u = 0.0;
        uLINK[i].ug = 0.0;
        uLINK[i].uef = 0.0;
        uLINK[i].u_joint = 0.0;
        uLINK[i].isPolygon=0;
                                                                                                
        
    }


    Status->dof=20+6;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5;
    Status->right_foot_ID=7;
    Status->left_foot_ID=13;
    Status->integral_R=0;
    Status->integral_L=0;
                            


    //Robot Configuration
    strcpy(uLINK[1].name, "BODY");
    uLINK[1].m  = 0 ;
    uLINK[1].color  = 3 ;
    uLINK[1].sister  = 0 ;//0
    uLINK[1].child  = RLEG ;
    uLINK[1].a(2) = 1;
    uLINK[1].p(2) = Lc+Lt+Lp;

    uLINK[1].R(0, 0) = 1;
    uLINK[1].R(1, 1) = 1;
    uLINK[1].R(2, 2) = 1;

    uLINK[1].c(0) = -0.00526;
    uLINK[1].c(2) = 0.09054;
    uLINK[1].I(0, 0) = 0.06714208509;
    uLINK[1].I(0, 1) = 0.00000021658;
    uLINK[1].I(0, 2) = -0.00057445503;
    uLINK[1].I(1, 0) = 0.00000021658;
    uLINK[1].I(1, 1) = 0.01982723311;
    uLINK[1].I(1, 2) = 0.00000091180;
    uLINK[1].I(2, 0) = -0.00057445503;
    uLINK[1].I(2, 1) = 0.00000091180;
    uLINK[1].I(2, 2) = 0.05554470420;



//uLINK(BODY).c = [-5.26, 0, 90.54]'*10^(-3);
//uLINK(BODY).I = [67142085.09,216.58,-574455.03;216.58,19827233.11,911.80;-574455.03,911.80,55544704.20]*10^(-9);



    strcpy(uLINK[RLEG].name, "RLEG_J0");
    uLINK[RLEG].m  = mPelvis/2 ;
    uLINK[RLEG].color  = 2 ;
    uLINK[RLEG].sister  = LLEG ;//8
    uLINK[RLEG].child  = RLEG+1 ;
    uLINK[RLEG].c(0) = -0.00526;
    uLINK[RLEG].c(2) = 0.09054;
    uLINK[RLEG].b(0) = Hx;
    uLINK[RLEG].b(1) = -Hy/2;
    uLINK[RLEG].a(0) = 1;
    uLINK[RLEG].I(0, 0) = 0.06714208509;
    uLINK[RLEG].I(0, 1) = 0.00000021658;
    uLINK[RLEG].I(0, 2) = -0.00057445503;
    uLINK[RLEG].I(1, 0) = 0.00000021658;
    uLINK[RLEG].I(1, 1) = 0.01982723311;
    uLINK[RLEG].I(1, 2) = 0.00000091180;
    uLINK[RLEG].I(2, 0) = -0.00057445503;
    uLINK[RLEG].I(2, 1) = 0.00000091180;
    uLINK[RLEG].I(2, 2) = 0.05554470420;


    strcpy(uLINK[RLEG+1].name, "RLEG_J1");
    uLINK[RLEG+1].m  = 0 ;
    uLINK[RLEG+1].color  = 2 ;
    uLINK[RLEG+1].sister  = 0 ;
    uLINK[RLEG+1].child  = RLEG+2 ;
    uLINK[RLEG+1].a(1) = 1;
    uLINK[RLEG+1].c(0) = 0.0068;
    uLINK[RLEG+1].c(1) = -0.0025;
    uLINK[RLEG+1].c(2) = 0.00076;
    uLINK[RLEG+1].I(0, 0) = 0.00004133363;
    uLINK[RLEG+1].I(0, 1) = -0.00000225009;
    uLINK[RLEG+1].I(0, 2) = -0.00000032224;
    uLINK[RLEG+1].I(1, 0) = -0.00000225009;
    uLINK[RLEG+1].I(1, 1) = 0.00003404560;
    uLINK[RLEG+1].I(1, 2) = 0.00000025384;
    uLINK[RLEG+1].I(2, 0) = -0.00000032224;
    uLINK[RLEG+1].I(2, 1) = 0.00000025384;
    uLINK[RLEG+1].I(2, 2) = 0.00006366584;

    strcpy(uLINK[RLEG+2].name, "RLEG_J2");
    uLINK[RLEG+2].m  = mUpperLimb ;
    uLINK[RLEG+2].color  = 2 ;
    uLINK[RLEG+2].sister  = 0 ;
    uLINK[RLEG+2].child  = RLEG+3 ;
    uLINK[RLEG+2].a(2) = 1;
    uLINK[RLEG+2].c(0) = 0.032;
    uLINK[RLEG+2].c(2) = -0.204;
    uLINK[RLEG+2].I(0, 0) = 0.08625887792;
    uLINK[RLEG+2].I(0, 1) = -0.00009806736;
    uLINK[RLEG+2].I(0, 2) = 0.00087486898;
    uLINK[RLEG+2].I(1, 0) = -0.00009806736;
    uLINK[RLEG+2].I(1, 1) = 0.08320117734;
    uLINK[RLEG+2].I(1, 2) = 0.00054829757;
    uLINK[RLEG+2].I(2, 0) = 0.00087486898;
    uLINK[RLEG+2].I(2, 1) = 0.00054829757;
    uLINK[RLEG+2].I(2, 2) = 0.01211293791;

    strcpy(uLINK[RLEG+3].name, "RLEG_J3");
    uLINK[RLEG+3].m  = mLowerLimb ;
    uLINK[RLEG+3].color  = 0 ;
    uLINK[RLEG+3].sister  = 0 ;
    uLINK[RLEG+3].child  = RLEG+4 ;
    uLINK[RLEG+3].a(1) = 1;
    uLINK[RLEG+3].b(2) = -Lc;
    uLINK[RLEG+3].c(2) = -0.255;
    uLINK[RLEG+3].I(0, 0) = 0.19372258067;
    uLINK[RLEG+3].I(0, 1) = 0.00000963489;
    uLINK[RLEG+3].I(0, 2) = 0.02878549993;
    uLINK[RLEG+3].I(1, 0) = 0.00000963489;
    uLINK[RLEG+3].I(1, 1) = 0.19983382882;
    uLINK[RLEG+3].I(1, 2) = -0.00125880365;
    uLINK[RLEG+3].I(2, 0) = 0.02878549993;
    uLINK[RLEG+3].I(2, 1) = -0.00125880365;
    uLINK[RLEG+3].I(2, 2) = 0.03540246245;

    strcpy(uLINK[RLEG+4].name, "RLEG_J4");
    uLINK[RLEG+4].m  = 0 ;
    uLINK[RLEG+4].color  = 1 ;
    uLINK[RLEG+4].sister  = 0 ;
    uLINK[RLEG+4].child  = RLEG+5 ;
    uLINK[RLEG+4].a(1) = 1;
    uLINK[RLEG+4].b(2) = -Lt;
    //uLINK[6].c(2) = -0.255;

    strcpy(uLINK[RLEG+5].name, "RLEG_J5");
    uLINK[RLEG+5].m  = mFoot ;
    uLINK[RLEG+5].color  = 1 ;
    uLINK[RLEG+5].sister  = 0 ;
    uLINK[RLEG+5].child  = 0 ;
    uLINK[RLEG+5].a(0) = 1;

    uLINK[RLEG+5].c(0) = 0.0859;
    uLINK[RLEG+5].c(2) = -0.0671;
    uLINK[RLEG+5].I(0, 0) = 0.00354735101;
    uLINK[RLEG+5].I(0, 1) = -0.00001373757;
    uLINK[RLEG+5].I(0, 2) = -0.00135282832;
    uLINK[RLEG+5].I(1, 0) = -0.00001373757;
    uLINK[RLEG+5].I(1, 1) = 0.00736336832;
    uLINK[RLEG+5].I(1, 2) = 0.00000176965;
    uLINK[RLEG+5].I(2, 0) = -0.00135282832;
    uLINK[RLEG+5].I(2, 1) = 0.00000176965;
    uLINK[RLEG+5].I(2, 2) = 0.00555234777;




    strcpy(uLINK[LLEG].name, "LLEG_J0");
    uLINK[LLEG].m  = mPelvis/2 ;
    uLINK[LLEG].color  = 2 ;
    uLINK[LLEG].sister  = SPINE ;
    uLINK[LLEG].child  = LLEG+1 ;
    uLINK[LLEG].c(0) = -0.00526;
    uLINK[LLEG].c(2) = 0.09054;
    uLINK[LLEG].a(0) = 1;
    uLINK[LLEG].b(0) = Hx;
    uLINK[LLEG].b(1) = Hy/2;
    uLINK[LLEG].I(0, 0) = 0.06714208509;
    uLINK[LLEG].I(0, 1) = 0.00000021658;
    uLINK[LLEG].I(0, 2) = -0.00057445503;
    uLINK[LLEG].I(1, 0) = 0.00000021658;
    uLINK[LLEG].I(1, 1) = 0.01982723311;
    uLINK[LLEG].I(1, 2) = 0.00000091180;
    uLINK[LLEG].I(2, 0) = -0.00057445503;
    uLINK[LLEG].I(2, 1) = 0.00000091180;
    uLINK[LLEG].I(2, 2) = 0.05554470420;


    strcpy(uLINK[LLEG+1].name, "LLEG_J1");
    uLINK[LLEG+1].m  = 0 ;
    uLINK[LLEG+1].color  = 2 ;
    uLINK[LLEG+1].sister  = 0 ;
    uLINK[LLEG+1].child  = LLEG+2 ;
    uLINK[LLEG+1].a(1) = 1;
    uLINK[LLEG+1].c(0) = 0.0068;
    uLINK[LLEG+1].c(1) = 0.0025;
    uLINK[LLEG+1].c(2) = 0.00076;
    uLINK[LLEG+1].I(0, 0) = 0.00004133363;
    uLINK[LLEG+1].I(0, 1) = -0.00000225009;
    uLINK[LLEG+1].I(0, 2) = -0.00000032224;
    uLINK[LLEG+1].I(1, 0) = -0.00000225009;
    uLINK[LLEG+1].I(1, 1) = 0.00003404560;
    uLINK[LLEG+1].I(1, 2) = 0.00000025384;
    uLINK[LLEG+1].I(2, 0) = -0.00000032224;
    uLINK[LLEG+1].I(2, 1) = 0.00000025384;
    uLINK[LLEG+1].I(2, 2) = 0.00006366584;

    strcpy(uLINK[LLEG+2].name, "LLEG_J2");
    uLINK[LLEG+2].m  = mUpperLimb ;
    uLINK[LLEG+2].color  = 2 ;
    uLINK[LLEG+2].sister  = 0 ;
    uLINK[LLEG+2].child  = LLEG+3 ;
    uLINK[LLEG+2].a(2) = 1;
    uLINK[LLEG+2].c(0) = 0.032;
    uLINK[LLEG+2].c(2) = -0.204;
    uLINK[LLEG+2].I(0, 0) = 0.08625887792;
    uLINK[LLEG+2].I(0, 1) = -0.00009806736;
    uLINK[LLEG+2].I(0, 2) = 0.00087486898;
    uLINK[LLEG+2].I(1, 0) = -0.00009806736;
    uLINK[LLEG+2].I(1, 1) = 0.08320117734;
    uLINK[LLEG+2].I(1, 2) = 0.00054829757;
    uLINK[LLEG+2].I(2, 0) = 0.00087486898;
    uLINK[LLEG+2].I(2, 1) = 0.00054829757;
    uLINK[LLEG+2].I(2, 2) = 0.01211293791;

    strcpy(uLINK[LLEG+3].name, "LLEG_J3");
    uLINK[LLEG+3].m  = mLowerLimb ;
    uLINK[LLEG+3].color  = 0 ;
    uLINK[LLEG+3].sister  = 0 ;
    uLINK[LLEG+3].child  = LLEG+4 ;
    uLINK[LLEG+3].a(1) = 1;
    uLINK[LLEG+3].b(2) = -Lc;
    uLINK[LLEG+3].c(2) = -0.255;
    uLINK[LLEG+3].I(0, 0) = 0.19372258067;
    uLINK[LLEG+3].I(0, 1) = 0.00000963489;
    uLINK[LLEG+3].I(0, 2) = 0.02878549993;
    uLINK[LLEG+3].I(1, 0) = 0.00000963489;
    uLINK[LLEG+3].I(1, 1) = 0.19983382882;
    uLINK[LLEG+3].I(1, 2) = -0.00125880365;
    uLINK[LLEG+3].I(2, 0) = 0.02878549993;
    uLINK[LLEG+3].I(2, 1) = -0.00125880365;
    uLINK[LLEG+3].I(2, 2) = 0.03540246245;

    strcpy(uLINK[LLEG+4].name, "LLEG_J4");
    uLINK[LLEG+4].m  = 0 ;
    uLINK[LLEG+4].color  = 1 ;
    uLINK[LLEG+4].sister  = 0 ;
    uLINK[LLEG+4].child  = LLEG+5 ;
    uLINK[LLEG+4].a(1) = 1;
    uLINK[LLEG+4].b(2) = -Lt;
    //uLINK[LLEG+4].c(2) = -0.255;

    strcpy(uLINK[LLEG+5].name, "LLEG_J5");
    uLINK[LLEG+5].m  = mFoot ;
    uLINK[LLEG+5].color  = 1 ;
    uLINK[LLEG+5].sister  = 0 ;
    uLINK[LLEG+5].child  = 0 ;
    uLINK[LLEG+5].a(0) = 1;
    uLINK[LLEG+5].c(0) = 0.0859;
    uLINK[LLEG+5].c(2) = -0.0671;
    uLINK[LLEG+5].I(0, 0) = 0.00354735101;
    uLINK[LLEG+5].I(0, 1) = -0.00001373757;
    uLINK[LLEG+5].I(0, 2) = -0.00135282832;
    uLINK[LLEG+5].I(1, 0) = -0.00001373757;
    uLINK[LLEG+5].I(1, 1) = 0.00736336832;
    uLINK[LLEG+5].I(1, 2) = 0.00000176965;
    uLINK[LLEG+5].I(2, 0) = -0.00135282832;
    uLINK[LLEG+5].I(2, 1) = 0.00000176965;
    uLINK[LLEG+5].I(2, 2) = 0.00555234777;

//------------------------------------------------------------------
//------------------------------------------------------------------
//------------------------------------------------------------------


    strcpy(uLINK[SPINE].name, "SPINE_J0");
    uLINK[SPINE].m  = 10 ;
    uLINK[SPINE].color  = 4 ;
    uLINK[SPINE].sister  = 0 ;//8
    uLINK[SPINE].child  = SPINE+1 ;
    //uLINK[SPINE].c(0) = -0.00526;
    //uLINK[SPINE].c(2) = 0.09054;
    //uLINK[SPINE].b(0) = Hx;
    uLINK[SPINE].b(2) = 0.3;
    uLINK[SPINE].a(1) = 1;
    uLINK[SPINE].I(0, 0) = 0.06714208509;
    uLINK[SPINE].I(0, 1) = 0.00000021658;
    uLINK[SPINE].I(0, 2) = -0.00057445503;
    uLINK[SPINE].I(1, 0) = 0.00000021658;
    uLINK[SPINE].I(1, 1) = 0.01982723311;
    uLINK[SPINE].I(1, 2) = 0.00000091180;
    uLINK[SPINE].I(2, 0) = -0.00057445503;
    uLINK[SPINE].I(2, 1) = 0.00000091180;
    uLINK[SPINE].I(2, 2) = 0.05554470420;


    strcpy(uLINK[SPINE+1].name, "SPINE_J1");
    uLINK[SPINE+1].m  = 0 ;
    uLINK[SPINE+1].color  = 4 ;
    uLINK[SPINE+1].sister  =  0 ;
    uLINK[SPINE+1].child  = RARM ;
    uLINK[SPINE+1].a(2) = 1;
    //uLINK[SPINE+1].c(0) = 0.0068;
    //uLINK[SPINE+1].c(1) = -0.0025;
    //uLINK[SPINE+1].c(2) = 0.00076;
    uLINK[SPINE+1].I(0, 0) = 0.00004133363;
    uLINK[SPINE+1].I(0, 1) = -0.00000225009;
    uLINK[SPINE+1].I(0, 2) = -0.00000032224;
    uLINK[SPINE+1].I(1, 0) = -0.00000225009;
    uLINK[SPINE+1].I(1, 1) = 0.00003404560;
    uLINK[SPINE+1].I(1, 2) = 0.00000025384;
    uLINK[SPINE+1].I(2, 0) = -0.00000032224;
    uLINK[SPINE+1].I(2, 1) = 0.00000025384;
    uLINK[SPINE+1].I(2, 2) = 0.00006366584;


    strcpy(uLINK[RARM].name, "RARM_J0");
    uLINK[RARM].m  = 5 ;
    uLINK[RARM].color  = 5 ;
    uLINK[RARM].sister  = LARM ;//8
    uLINK[RARM].child  = RARM+1 ;
    //uLINK[RARM].c(0) = -0.00526;
    //uLINK[RARM].c(2) = 0.09054;
    uLINK[RARM].b(1) = -0.2;
    uLINK[RARM].b(2) = 0.4;
    uLINK[RARM].a(0) = 1;
    uLINK[RARM].I(0, 0) = 0.06714208509;
    uLINK[RARM].I(0, 1) = 0.00000021658;
    uLINK[RARM].I(0, 2) = -0.00057445503;
    uLINK[RARM].I(1, 0) = 0.00000021658;
    uLINK[RARM].I(1, 1) = 0.01982723311;
    uLINK[RARM].I(1, 2) = 0.00000091180;
    uLINK[RARM].I(2, 0) = -0.00057445503;
    uLINK[RARM].I(2, 1) = 0.00000091180;
    uLINK[RARM].I(2, 2) = 0.05554470420;


    strcpy(uLINK[RARM+1].name, "RARM_J1");
    uLINK[RARM+1].m  = 0 ;
    uLINK[RARM+1].color  = 5 ;
    uLINK[RARM+1].sister  = 0 ;
    uLINK[RARM+1].child  = RARM+2 ;
    uLINK[RARM+1].a(1) = 1;
    //uLINK[RARM+1].c(0) = 0.0068;
    //uLINK[RARM+1].c(1) = -0.0025;
    //uLINK[RARM+1].c(2) = 0.00076;
    uLINK[RARM+1].I(0, 0) = 0.00004133363;
    uLINK[RARM+1].I(0, 1) = -0.00000225009;
    uLINK[RARM+1].I(0, 2) = -0.00000032224;
    uLINK[RARM+1].I(1, 0) = -0.00000225009;
    uLINK[RARM+1].I(1, 1) = 0.00003404560;
    uLINK[RARM+1].I(1, 2) = 0.00000025384;
    uLINK[RARM+1].I(2, 0) = -0.00000032224;
    uLINK[RARM+1].I(2, 1) = 0.00000025384;
    uLINK[RARM+1].I(2, 2) = 0.00006366584;

    strcpy(uLINK[RARM+2].name, "RARM_J2");
    uLINK[RARM+2].m  = 10 ;
    uLINK[RARM+2].color  = 6 ;
    uLINK[RARM+2].sister  = 0 ;
    uLINK[RARM+2].child  = 0 ;
    uLINK[RARM+2].a(1) = 1;
    uLINK[RARM+2].b(2) = -0.4;
    //uLINK[RARM+2].c(0) = 0.032;
    //uLINK[RARM+2].c(2) = -0.204;
    uLINK[RARM+2].I(0, 0) = 0.08625887792;
    uLINK[RARM+2].I(0, 1) = -0.00009806736;
    uLINK[RARM+2].I(0, 2) = 0.00087486898;
    uLINK[RARM+2].I(1, 0) = -0.00009806736;
    uLINK[RARM+2].I(1, 1) = 0.08320117734;
    uLINK[RARM+2].I(1, 2) = 0.00054829757;
    uLINK[RARM+2].I(2, 0) = 0.00087486898;
    uLINK[RARM+2].I(2, 1) = 0.00054829757;
    uLINK[RARM+2].I(2, 2) = 0.01211293791;







    strcpy(uLINK[LARM].name, "LARM_J0");
    uLINK[LARM].m  = 5 ;
    uLINK[LARM].color  = 5 ;
    uLINK[LARM].sister  = 0 ;
    uLINK[LARM].child  = LARM+1 ;
    //uLINK[LARM].c(0) = -0.00526;
    //uLINK[LARM].c(2) = 0.09054;
    uLINK[LARM].a(0) = 1;
    uLINK[LARM].b(1) = 0.2;
    uLINK[LARM].b(2) = 0.4;
    uLINK[LARM].I(0, 0) = 0.06714208509;
    uLINK[LARM].I(0, 1) = 0.00000021658;
    uLINK[LARM].I(0, 2) = -0.00057445503;
    uLINK[LARM].I(1, 0) = 0.00000021658;
    uLINK[LARM].I(1, 1) = 0.01982723311;
    uLINK[LARM].I(1, 2) = 0.00000091180;
    uLINK[LARM].I(2, 0) = -0.00057445503;
    uLINK[LARM].I(2, 1) = 0.00000091180;
    uLINK[LARM].I(2, 2) = 0.05554470420;


    strcpy(uLINK[LARM+1].name, "LARM_J1");
    uLINK[LARM+1].m  = 0 ;
    uLINK[LARM+1].color  = 5 ;
    uLINK[LARM+1].sister  = 0 ;
    uLINK[LARM+1].child  = LARM+2 ;
    uLINK[LARM+1].a(1) = 1;
    //uLINK[LARM+1].c(0) = 0.0068;
    //uLINK[LARM+1].c(1) = 0.0025;
    //uLINK[LARM+1].c(2) = 0.00076;
    uLINK[LARM+1].I(0, 0) = 0.00004133363;
    uLINK[LARM+1].I(0, 1) = -0.00000225009;
    uLINK[LARM+1].I(0, 2) = -0.00000032224;
    uLINK[LARM+1].I(1, 0) = -0.00000225009;
    uLINK[LARM+1].I(1, 1) = 0.00003404560;
    uLINK[LARM+1].I(1, 2) = 0.00000025384;
    uLINK[LARM+1].I(2, 0) = -0.00000032224;
    uLINK[LARM+1].I(2, 1) = 0.00000025384;
    uLINK[LARM+1].I(2, 2) = 0.00006366584;

    strcpy(uLINK[LARM+2].name, "LARM_J2");
    uLINK[LARM+2].m  = 10 ;
    uLINK[LARM+2].color  = 6 ;
    uLINK[LARM+2].sister  = 0 ;
    uLINK[LARM+2].child  = 0 ;
    uLINK[LARM+2].a(1) = 1;
    uLINK[LARM+2].b(2) = -0.4;
    //uLINK[LARM+2].c(0) = 0.032;
    //uLINK[LARM+2].c(2) = -0.204;
    uLINK[LARM+2].I(0, 0) = 0.08625887792;
    uLINK[LARM+2].I(0, 1) = -0.00009806736;
    uLINK[LARM+2].I(0, 2) = 0.00087486898;
    uLINK[LARM+2].I(1, 0) = -0.00009806736;
    uLINK[LARM+2].I(1, 1) = 0.08320117734;
    uLINK[LARM+2].I(1, 2) = 0.00054829757;
    uLINK[LARM+2].I(2, 0) = 0.00087486898;
    uLINK[LARM+2].I(2, 1) = 0.00054829757;
    uLINK[LARM+2].I(2, 2) = 0.01211293791;







    // Contacts
//    uLINK[1].shape[0]=0.1;
//    uLINK[1].shape[1]=0.3;
//    uLINK[1].shape[2]=0.04;
//    uLINK[1].com[0]=0;
//    uLINK[1].com[1]=0;
//    uLINK[1].com[2]=0.05;
//    SetupRigidBody(uLINK,1);
//    uLINK[1].supportHeight=0.0;

    uLINK[RLEG+5].shape[0]=0.28;
    uLINK[RLEG+5].shape[1]=0.08;
    uLINK[RLEG+5].shape[2]=0.04;
    uLINK[RLEG+5].com[0]=0.0853;
    uLINK[RLEG+5].com[1]=0;
    uLINK[RLEG+5].com[2]=-0.09;
    SetupRigidBody(uLINK,RLEG+5);
    uLINK[RLEG+5].supportHeight=0.0;

    uLINK[LLEG+5].shape[0]=0.28;
    uLINK[LLEG+5].shape[1]=0.08;
    uLINK[LLEG+5].shape[2]=0.04;
    uLINK[LLEG+5].com[0]=0.0853;
    uLINK[LLEG+5].com[1]=0;
    uLINK[LLEG+5].com[2]=-0.09;
    SetupRigidBody(uLINK,LLEG+5);
    uLINK[LLEG+5].supportHeight=0.0;



    uLINK[RARM+2].shape[0]=0.04;
    uLINK[RARM+2].shape[1]=0.04;
    uLINK[RARM+2].shape[2]=0.25;
    uLINK[RARM+2].com[0]=0.0;
    uLINK[RARM+2].com[1]=0;
    uLINK[RARM+2].com[2]=-0.15;
    SetupRigidBody(uLINK,RARM+2);
    uLINK[RARM+2].supportHeight=0.0;

    uLINK[LARM+2].shape[0]=0.04;
    uLINK[LARM+2].shape[1]=0.04;
    uLINK[LARM+2].shape[2]=0.25;
    uLINK[LARM+2].com[0]=0.0;
    uLINK[LARM+2].com[1]=0;
    uLINK[LARM+2].com[2]=-0.15;
    SetupRigidBody(uLINK,LARM+2);
    uLINK[LARM+2].supportHeight=0.0;


    uLINK[SPINE+1].shape[0]=0.05;
    uLINK[SPINE+1].shape[1]=0.33;
    uLINK[SPINE+1].shape[2]=0.4;
    uLINK[SPINE+1].com[0]=0;
    uLINK[SPINE+1].com[1]=0;
    uLINK[SPINE+1].com[2]=0.25;
    SetupRigidBody(uLINK,SPINE+1);
    uLINK[SPINE+1].supportHeight=0.0;



    FindMother(uLINK,Status,1);

    ForwardKinematics(uLINK,1);

    ForwardVelocity(uLINK,1);

    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    CalcCoM(uLINK,com);
    Status->com_old = com;
    

}
#endif



