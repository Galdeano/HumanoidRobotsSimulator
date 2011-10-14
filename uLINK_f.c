#include <string.h>

#include "d:\Mb\StLib\Mat.h"

#include "uLINK_f.h"

#include "FindMother_f.h"
#include "ForwardKinematics_f.h"
//#include "ForwardVelocity.h"

#include "Setup.h"

#if Sherpa
void SetupRobot_f(Struct_uLINK uLINK[],Struct_State *Status)
{


    float Hy=0.31f;
    float Hx=0.0468f;
    float Lc=0.5073f;
    float Lt=0.510f;
    float Lp=0.1104f;

    // Eléments d'inertie du robot Sherpa - 20091021SK
    float mPelvis=6.1f; // [kg]
    float mUpperLimb=8.f;
    float mLowerLimb=10.9f;
    float mFoot=1.6f;
    //Total=mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot
    //printf("%8.4f \n",mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot);


    int i;
    for(i=1; i<NbLinks; i++)
    {
        uLINK[i].q = 0.f;
        uLINK[i].dq = 0.f;
        uLINK[i].ddq = 0.f;
        uLINK[i].u = 0.f;
        uLINK[i].ug = 0.f;
        uLINK[i].uef = 0.f;
        uLINK[i].u_joint = 0.f;

        MatClrf(uLINK[i].a,3,1);
        MatClrf(uLINK[i].b,3,1);
        MatClrf(uLINK[i].c,3,1);
        MatClrf(uLINK[i].p,3,1);
        MatClrf(uLINK[i].hw,3,1);
        MatClrf(uLINK[i].hv,3,1);

        //MatClrf(uLINK[i].R,3,3);
    }


    Status->ddl=18;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5f;
    Status->right_foot_ID=7;
    Status->left_foot_ID=13;




    //Robot Configuration
    strcpy(uLINK[1].name, "BODY");
    uLINK[1].m  = 0.f ;
    uLINK[1].sister  = 0 ;//0
    uLINK[1].child  = 2 ;
    uLINK[1].a[2] = 1.f;
    uLINK[1].p[2] = Lc+Lt+Lp;
    uLINK[1].c[0] = -0.00526f;
    uLINK[1].c[2] = 0.09054f;
    //MatIf( uLINK[1].R , 3 ) ;






    strcpy(uLINK[2].name, "RLEG_J0");
    uLINK[2].m  = mPelvis/2 ;
    uLINK[2].sister  = 8 ;//8
    uLINK[2].child  = 3 ;
    uLINK[2].a[0] = 1.f;
    uLINK[2].b[0] = Hx;
    uLINK[2].b[1] = -Hy/2;


    strcpy(uLINK[3].name, "RLEG_J1");
    uLINK[3].m  = 0.f ;
    uLINK[3].sister  = 0 ;
    uLINK[3].child  = 4 ;
    uLINK[3].a[1] = 1.f;
    uLINK[3].c[0] = 0.0068f;
    uLINK[3].c[1] = -0.0025f;
    uLINK[3].c[2] = 0.00076f;

    strcpy(uLINK[4].name, "RLEG_J2");
    uLINK[4].m  = mUpperLimb ;
    uLINK[4].sister  = 0 ;
    uLINK[4].child  = 5 ;
    uLINK[4].a[2] = 1.f;
    uLINK[4].c[0] = 0.032f;
    uLINK[4].c[2] = -0.204f;

    strcpy(uLINK[5].name, "RLEG_J3");
    uLINK[5].m  = mLowerLimb ;
    uLINK[5].sister  = 0 ;
    uLINK[5].child  = 6 ;
    uLINK[5].a[1] = 1.f;
    uLINK[5].c[2] = -0.255f;
    uLINK[5].b[2] = -Lc;

    strcpy(uLINK[6].name, "RLEG_J4");
    uLINK[6].m  = 0.f ;
    uLINK[6].sister  = 0 ;
    uLINK[6].child  = 7 ;
    uLINK[6].a[1] = 1.f;
    uLINK[6].b[2] = -Lt;

    strcpy(uLINK[7].name, "RLEG_J5");
    uLINK[7].m  = mFoot ;
    uLINK[7].sister  = 0 ;
    uLINK[7].child  = 0 ;
    uLINK[7].a[0] = 1.f;
    uLINK[7].c[0] = 0.0859f;
    uLINK[7].c[2] = -0.0671f;




    strcpy(uLINK[8].name, "LLEG_J0");
    uLINK[8].m  = mPelvis/2 ;
    uLINK[8].sister  = 0 ;
    uLINK[8].child  = 9 ;
    uLINK[8].a[0] = 1.f;
    uLINK[8].b[0] = Hx;
    uLINK[8].b[1] = Hy/2;

    strcpy(uLINK[9].name, "LLEG_J1");
    uLINK[9].m  = 0.f ;
    uLINK[9].sister  = 0 ;
    uLINK[9].child  = 10 ;
    uLINK[9].a[1] = 1.f;
    uLINK[9].c[0] = 0.0068f;
    uLINK[9].c[1] = 0.0025f;
    uLINK[9].c[2] = 0.00076f;

    strcpy(uLINK[10].name, "LLEG_J2");
    uLINK[10].m  = mUpperLimb ;
    uLINK[10].sister  = 0 ;
    uLINK[10].child  = 11 ;
    uLINK[10].a[2] = 1.f;
    uLINK[10].c[0] = 0.032f;
    uLINK[10].c[2] = -0.204f;

    strcpy(uLINK[11].name, "LLEG_J3");
    uLINK[11].m  = mLowerLimb ;
    uLINK[11].sister  = 0 ;
    uLINK[11].child  = 12 ;
    uLINK[11].a[1] = 1.f;
    uLINK[11].c[2] = -0.255f;
    uLINK[11].b[2] = -Lc;

    strcpy(uLINK[12].name, "LLEG_J4");
    uLINK[12].m  = 0.f ;
    uLINK[12].sister  = 0 ;
    uLINK[12].child  = 13 ;
    uLINK[12].a[1] = 1.f;
    uLINK[12].b[2] = -Lt;

    strcpy(uLINK[13].name, "LLEG_J5");
    uLINK[13].m  = mFoot ;
    uLINK[13].sister  = 0 ;
    uLINK[13].child  = 0 ;
    uLINK[13].a[0] = 1.f;
    uLINK[13].c[0] = 0.0859f;
    uLINK[13].c[2] = -0.0671f;



    FindMother_f(uLINK,1);

    ForwardKinematics_f(uLINK,1);

    //ForwardVelocity(uLINK,1);


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


#if Generic
void SetupRobot_f(Struct_uLINK uLINK[],Struct_State *Status)
{

    float Hy=0.31f;
    float Hx=0.0468f;
    float Lc=0.5073f;
    float Lt=0.510f;
    float Lp=0.1104f;

    // Eléments d'inertie du robot Sherpa - 20091021SK
    float mPelvis=6.1f; // [kg]
    float mUpperLimb=8.f;
    float mLowerLimb=10.9f;
    float mFoot=1.6f;
    //Total=mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot
    //printf("%8.4f \n",mPelvis+2*mUpperLimb+2*mLowerLimb+2*mFoot);


    int i;
    for(i=1; i<NbLinks; i++)
    {
        uLINK[i].q = 0.f;
        uLINK[i].dq = 0.f;
        uLINK[i].ddq = 0.f;
        uLINK[i].u = 0.f;
        uLINK[i].ug = 0.f;
        uLINK[i].uef = 0.f;
        uLINK[i].u_joint = 0.f;

        MatClrf(uLINK[i].a,3,1);
        MatClrf(uLINK[i].b,3,1);
        MatClrf(uLINK[i].c,3,1);
        MatClrf(uLINK[i].p,3,1);
        MatClrf(uLINK[i].hw,3,1);
        MatClrf(uLINK[i].hv,3,1);

        //MatClrf(uLINK[i].R,3,3);
    }


    Status->ddl=26;
    Status->support=0; //0:none,1:right,2:left,3:both
    Status->desired_support=0;
    Status->distribution_y=0.5f;
    Status->right_foot_ID=7;
    Status->left_foot_ID=13;




    //Robot Configuration
    strcpy(uLINK[1].name, "BODY");
    uLINK[1].m  = 0.f ;
    uLINK[1].sister  = 0 ;//0
    uLINK[1].child  = RLEG ;
    uLINK[1].a[2] = 1.f;
    uLINK[1].p[2] = Lc+Lt+Lp;
    uLINK[1].c[0] = -0.00526f;
    uLINK[1].c[2] = 0.09054f;
    //MatIf( uLINK[1].R , 3 ) ;






    strcpy(uLINK[RLEG].name, "RLEG_J0");
    uLINK[RLEG].m  = mPelvis/2 ;
    uLINK[RLEG].sister  = LLEG ;//8
    uLINK[RLEG].child  = RLEG+1 ;
    uLINK[RLEG].a[0] = 1.f;
    uLINK[RLEG].b[0] = Hx;
    uLINK[RLEG].b[1] = -Hy/2;


    strcpy(uLINK[RLEG+1].name, "RLEG_J1");
    uLINK[RLEG+1].m  = 0.f ;
    uLINK[RLEG+1].sister  = 0 ;
    uLINK[RLEG+1].child  = RLEG+2 ;
    uLINK[RLEG+1].a[1] = 1.f;
    uLINK[RLEG+1].c[0] = 0.0068f;
    uLINK[RLEG+1].c[1] = -0.0025f;
    uLINK[RLEG+1].c[2] = 0.00076f;

    strcpy(uLINK[RLEG+2].name, "RLEG_J2");
    uLINK[RLEG+2].m  = mUpperLimb ;
    uLINK[RLEG+2].sister  = 0 ;
    uLINK[RLEG+2].child  = RLEG+3 ;
    uLINK[RLEG+2].a[2] = 1.f;
    uLINK[RLEG+2].c[0] = 0.032f;
    uLINK[RLEG+2].c[2] = -0.204f;

    strcpy(uLINK[RLEG+3].name, "RLEG_J3");
    uLINK[RLEG+3].m  = mLowerLimb ;
    uLINK[RLEG+3].sister  = 0 ;
    uLINK[RLEG+3].child  = RLEG+4 ;
    uLINK[RLEG+3].a[1] = 1.f;
    uLINK[RLEG+3].c[2] = -0.255f;
    uLINK[RLEG+3].b[2] = -Lc;

    strcpy(uLINK[RLEG+4].name, "RLEG_J4");
    uLINK[RLEG+4].m  = 0.f ;
    uLINK[RLEG+4].sister  = 0 ;
    uLINK[RLEG+4].child  = RLEG+5 ;
    uLINK[RLEG+4].a[1] = 1.f;
    uLINK[RLEG+4].b[2] = -Lt;

    strcpy(uLINK[RLEG+5].name, "RLEG_J5");
    uLINK[RLEG+5].m  = mFoot ;
    uLINK[RLEG+5].sister  = 0 ;
    uLINK[RLEG+5].child  = 0 ;
    uLINK[RLEG+5].a[0] = 1.f;
    uLINK[RLEG+5].c[0] = 0.0859f;
    uLINK[RLEG+5].c[2] = -0.0671f;




    strcpy(uLINK[LLEG].name, "LLEG_J0");
    uLINK[LLEG].m  = mPelvis/2 ;
    uLINK[LLEG].sister  = SPINE ;
    uLINK[LLEG].child  = LLEG+1 ;
    uLINK[LLEG].a[0] = 1.f;
    uLINK[LLEG].b[0] = Hx;
    uLINK[LLEG].b[1] = Hy/2;

    strcpy(uLINK[LLEG+1].name, "LLEG_J1");
    uLINK[LLEG+1].m  = 0.f ;
    uLINK[LLEG+1].sister  = 0 ;
    uLINK[LLEG+1].child  = LLEG+2 ;
    uLINK[LLEG+1].a[1] = 1.f;
    uLINK[LLEG+1].c[0] = 0.0068f;
    uLINK[LLEG+1].c[1] = 0.0025f;
    uLINK[LLEG+1].c[2] = 0.00076f;

    strcpy(uLINK[LLEG+2].name, "LLEG_J2");
    uLINK[LLEG+2].m  = mUpperLimb ;
    uLINK[LLEG+2].sister  = 0 ;
    uLINK[LLEG+2].child  = LLEG+3 ;
    uLINK[LLEG+2].a[2] = 1.f;
    uLINK[LLEG+2].c[0] = 0.032f;
    uLINK[LLEG+2].c[2] = -0.204f;

    strcpy(uLINK[LLEG+3].name, "LLEG_J3");
    uLINK[LLEG+3].m  = mLowerLimb ;
    uLINK[LLEG+3].sister  = 0 ;
    uLINK[LLEG+3].child  = LLEG+4 ;
    uLINK[LLEG+3].a[1] = 1.f;
    uLINK[LLEG+3].c[2] = -0.255f;
    uLINK[LLEG+3].b[2] = -Lc;

    strcpy(uLINK[LLEG+4].name, "LLEG_J4");
    uLINK[LLEG+4].m  = 0.f ;
    uLINK[LLEG+4].sister  = 0 ;
    uLINK[LLEG+4].child  = LLEG+5 ;
    uLINK[LLEG+4].a[1] = 1.f;
    uLINK[LLEG+4].b[2] = -Lt;

    strcpy(uLINK[LLEG+5].name, "LLEG_J5");
    uLINK[LLEG+5].m  = mFoot ;
    uLINK[LLEG+5].sister  = 0 ;
    uLINK[LLEG+5].child  = 0 ;
    uLINK[LLEG+5].a[0] = 1.f;
    uLINK[LLEG+5].c[0] = 0.0859f;
    uLINK[LLEG+5].c[2] = -0.0671f;



    strcpy(uLINK[SPINE].name, "SPINE_J0");
    uLINK[SPINE].m  = 10.f ;
    uLINK[SPINE].sister  = 0 ;//8
    uLINK[SPINE].child  = SPINE+1 ;
    uLINK[SPINE].a[1] = 1.f;
    uLINK[SPINE].b[2] = 0.3;

    strcpy(uLINK[SPINE+1].name, "SPINE_J1");
    uLINK[SPINE+1].m  = 0.f ;
    uLINK[SPINE+1].sister  = RARM ;
    uLINK[SPINE+1].child  = 0 ;
    uLINK[SPINE+1].a[2] = 1.f;




    strcpy(uLINK[RARM].name, "RARM_J0");
    uLINK[RARM].m  = 5.f ;
    uLINK[RARM].sister  = LARM ;
    uLINK[RARM].child  = RARM+1 ;
    uLINK[RARM].a[0] = 1.f;
    uLINK[RARM].b[1] = -0.2;
    uLINK[RARM].b[2] = 0.4;

    strcpy(uLINK[RARM+1].name, "RARM_J1");
    uLINK[RARM+1].m  = 0.f ;
    uLINK[RARM+1].sister  = 0 ;
    uLINK[RARM+1].child  = RARM+2 ;
    uLINK[RARM+1].a[1] = 1.f;

    strcpy(uLINK[RARM+2].name, "RARM_J2");
    uLINK[RARM+2].m  = 10.f ;
    uLINK[RARM+2].sister  = 0 ;
    uLINK[RARM+2].child  = 0 ;
    uLINK[RARM+2].a[1] = 1.f;
    uLINK[RARM+2].b[2] = -0.4;



    strcpy(uLINK[LARM].name, "LARM_J0");
    uLINK[LARM].m  = 5.f ;
    uLINK[LARM].sister  = 0 ;
    uLINK[LARM].child  = LARM+1 ;
    uLINK[LARM].a[0] = 1.f;
    uLINK[LARM].b[1] = 0.2;
    uLINK[LARM].b[2] = 0.4;

    strcpy(uLINK[LARM+1].name, "LARM_J1");
    uLINK[LARM+1].m  = 0.f ;
    uLINK[LARM+1].sister  = 0 ;
    uLINK[LARM+1].child  = LARM+2 ;
    uLINK[LARM+1].a[1] = 1.f;

    strcpy(uLINK[LARM+2].name, "LARM_J2");
    uLINK[LARM+2].m  = 10.f ;
    uLINK[LARM+2].sister  = 0 ;
    uLINK[LARM+2].child  = 0 ;
    uLINK[LARM+2].a[1] = 1.f;
    uLINK[LARM+2].b[2] = -0.4;





    FindMother_f(uLINK,1);

    ForwardKinematics_f(uLINK,1);

}
#endif


