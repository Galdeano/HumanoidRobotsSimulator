#include <Eigen/Dense>



#include <stdio.h>
#include <string.h>

#include "uLink.h"
#include "SetupRigidBody.h"
#include "FindMother.h"
#include "ForwardKinematics.h"
#include "ForwardVelocity.h"
#include "CalcCoM.h"
#include "Setup.h"
#include "LoadSherpa.h"


void LoadSherpa(SuLINK uLINK[],State *Status)
{

    int i;
    for(i=1; i<14; i++)
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
    Status->com_old.setZero();
    Status->forCoP_R.setZero();
    Status->posCoP_R.setZero();
    Status->forCoP_L.setZero();
    Status->posCoP_L.setZero();
    Status->FootCenter_R.setZero();
    Status->FootCenter_L.setZero();


    strcpy(uLINK[1].name, "BODY");
    uLINK[1].m  = 6.100000 ;
    uLINK[1].fixed  = 0 ;
    uLINK[1].color  = 3 ;
    uLINK[1].sister  = 0 ;
    uLINK[1].child  = 2 ;
    uLINK[1].fixed  = 0 ;
    uLINK[1].qmin  = -1.570796 ;
    uLINK[1].qmax  = 1.570796 ;
    uLINK[1].qmoy  = 0.000000 ;
    uLINK[1].q  = 0.000000 ;
    uLINK[1].umin  = -2.000000 ;
    uLINK[1].umax  = 2.000000 ;
    uLINK[1].a(0) = 0.000000;
    uLINK[1].a(1) = 0.000000;
    uLINK[1].a(2) = 1.000000;
    uLINK[1].b(0) = 0.000000;
    uLINK[1].b(1) = 0.000000;
    uLINK[1].b(2) = 0.000000;
    uLINK[1].p(0) = 0.000000;
    uLINK[1].p(1) = 0.000000;
    uLINK[1].p(2) = 1.127700;

    uLINK[1].R(0, 0) = 1;
    uLINK[1].R(1, 1) = 1;
    uLINK[1].R(2, 2) = 1;

    uLINK[1].c(0) = -0.005260;
    uLINK[1].c(1) = 0.000000;
    uLINK[1].c(2) = 0.090540;
    uLINK[1].I(0, 0) = 0.067142;
    uLINK[1].I(0, 1) = 0.000000;
    uLINK[1].I(0, 2) = -0.000574;
    uLINK[1].I(1, 0) = 0.000000;
    uLINK[1].I(1, 1) = 0.019827;
    uLINK[1].I(1, 2) = 0.000001;
    uLINK[1].I(2, 0) = -0.000574;
    uLINK[1].I(2, 1) = 0.000001;
    uLINK[1].I(2, 2) = 0.055545;

    strcpy(uLINK[2].name, "RLEG_J0");
    uLINK[2].m  = 0.000000 ;
    uLINK[2].fixed  = 0 ;
    uLINK[2].color  = 2 ;
    uLINK[2].sister  = 8 ;
    uLINK[2].child  = 3 ;
    uLINK[2].fixed  = 0 ;
    uLINK[2].qmin  = -1.570796 ;
    uLINK[2].qmax  = 1.570796 ;
    uLINK[2].qmoy  = 0.000000 ;
    uLINK[2].q  = 0.000000 ;
    uLINK[2].umin  = -2.000000 ;
    uLINK[2].umax  = 2.000000 ;
    uLINK[2].a(0) = 1.000000;
    uLINK[2].a(1) = 0.000000;
    uLINK[2].a(2) = 0.000000;
    uLINK[2].b(0) = 0.046800;
    uLINK[2].b(1) = -0.155000;
    uLINK[2].b(2) = 0.000000;
    uLINK[2].p(0) = 0.046800;
    uLINK[2].p(1) = -0.155000;
    uLINK[2].p(2) = 1.127700;

    uLINK[2].R(0, 0) = 1;
    uLINK[2].R(1, 1) = 1;
    uLINK[2].R(2, 2) = 1;

    uLINK[2].c(0) = -0.005260;
    uLINK[2].c(1) = 0.000000;
    uLINK[2].c(2) = 0.090540;
    uLINK[2].I(0, 0) = 0.067142;
    uLINK[2].I(0, 1) = 0.000000;
    uLINK[2].I(0, 2) = -0.000574;
    uLINK[2].I(1, 0) = 0.000000;
    uLINK[2].I(1, 1) = 0.019827;
    uLINK[2].I(1, 2) = 0.000001;
    uLINK[2].I(2, 0) = -0.000574;
    uLINK[2].I(2, 1) = 0.000001;
    uLINK[2].I(2, 2) = 0.055545;

    strcpy(uLINK[3].name, "RLEG_J1");
    uLINK[3].m  = 0.000000 ;
    uLINK[3].fixed  = 0 ;
    uLINK[3].color  = 2 ;
    uLINK[3].sister  = 0 ;
    uLINK[3].child  = 4 ;
    uLINK[3].fixed  = 0 ;
    uLINK[3].qmin  = -1.570796 ;
    uLINK[3].qmax  = 1.570796 ;
    uLINK[3].qmoy  = -0.174533 ;
    uLINK[3].q  = -0.174533 ;
    uLINK[3].umin  = -2.000000 ;
    uLINK[3].umax  = 2.000000 ;
    uLINK[3].a(0) = 0.000000;
    uLINK[3].a(1) = 1.000000;
    uLINK[3].a(2) = 0.000000;
    uLINK[3].b(0) = 0.000000;
    uLINK[3].b(1) = 0.000000;
    uLINK[3].b(2) = 0.000000;
    uLINK[3].p(0) = 0.046800;
    uLINK[3].p(1) = -0.155000;
    uLINK[3].p(2) = 1.127700;

    uLINK[3].R(0, 0) = 1;
    uLINK[3].R(1, 1) = 1;
    uLINK[3].R(2, 2) = 1;

    uLINK[3].c(0) = 0.006800;
    uLINK[3].c(1) = -0.002500;
    uLINK[3].c(2) = 0.000760;
    uLINK[3].I(0, 0) = 0.000041;
    uLINK[3].I(0, 1) = -0.000002;
    uLINK[3].I(0, 2) = -0.000000;
    uLINK[3].I(1, 0) = -0.000002;
    uLINK[3].I(1, 1) = 0.000034;
    uLINK[3].I(1, 2) = 0.000000;
    uLINK[3].I(2, 0) = -0.000000;
    uLINK[3].I(2, 1) = 0.000000;
    uLINK[3].I(2, 2) = 0.000064;

    strcpy(uLINK[4].name, "RLEG_J2");
    uLINK[4].m  = 8.000000 ;
    uLINK[4].fixed  = 0 ;
    uLINK[4].color  = 2 ;
    uLINK[4].sister  = 0 ;
    uLINK[4].child  = 5 ;
    uLINK[4].fixed  = 0 ;
    uLINK[4].qmin  = -1.570796 ;
    uLINK[4].qmax  = 1.570796 ;
    uLINK[4].qmoy  = 0.000000 ;
    uLINK[4].q  = 0.000000 ;
    uLINK[4].umin  = -2.000000 ;
    uLINK[4].umax  = 2.000000 ;
    uLINK[4].a(0) = 0.000000;
    uLINK[4].a(1) = 0.000000;
    uLINK[4].a(2) = 1.000000;
    uLINK[4].b(0) = 0.000000;
    uLINK[4].b(1) = 0.000000;
    uLINK[4].b(2) = 0.000000;
    uLINK[4].p(0) = 0.046800;
    uLINK[4].p(1) = -0.155000;
    uLINK[4].p(2) = 1.127700;

    uLINK[4].R(0, 0) = 1;
    uLINK[4].R(1, 1) = 1;
    uLINK[4].R(2, 2) = 1;

    uLINK[4].c(0) = 0.032000;
    uLINK[4].c(1) = 0.000000;
    uLINK[4].c(2) = -0.204000;
    uLINK[4].I(0, 0) = 0.086259;
    uLINK[4].I(0, 1) = -0.000098;
    uLINK[4].I(0, 2) = 0.000875;
    uLINK[4].I(1, 0) = -0.000098;
    uLINK[4].I(1, 1) = 0.083201;
    uLINK[4].I(1, 2) = 0.000548;
    uLINK[4].I(2, 0) = 0.000875;
    uLINK[4].I(2, 1) = 0.000548;
    uLINK[4].I(2, 2) = 0.012113;

    strcpy(uLINK[5].name, "RLEG_J3");
    uLINK[5].m  = 10.900000 ;
    uLINK[5].fixed  = 0 ;
    uLINK[5].color  = 0 ;
    uLINK[5].sister  = 0 ;
    uLINK[5].child  = 6 ;
    uLINK[5].fixed  = 0 ;
    uLINK[5].qmin  = -1.570796 ;
    uLINK[5].qmax  = 1.570796 ;
    uLINK[5].qmoy  = 0.349066 ;
    uLINK[5].q  = 0.349066 ;
    uLINK[5].umin  = -2.000000 ;
    uLINK[5].umax  = 2.000000 ;
    uLINK[5].a(0) = 0.000000;
    uLINK[5].a(1) = 1.000000;
    uLINK[5].a(2) = 0.000000;
    uLINK[5].b(0) = 0.000000;
    uLINK[5].b(1) = 0.000000;
    uLINK[5].b(2) = -0.507300;
    uLINK[5].p(0) = 0.046800;
    uLINK[5].p(1) = -0.155000;
    uLINK[5].p(2) = 0.620400;

    uLINK[5].R(0, 0) = 1;
    uLINK[5].R(1, 1) = 1;
    uLINK[5].R(2, 2) = 1;

    uLINK[5].c(0) = 0.000000;
    uLINK[5].c(1) = 0.000000;
    uLINK[5].c(2) = -0.255000;
    uLINK[5].I(0, 0) = 0.193723;
    uLINK[5].I(0, 1) = 0.000010;
    uLINK[5].I(0, 2) = 0.028785;
    uLINK[5].I(1, 0) = 0.000010;
    uLINK[5].I(1, 1) = 0.199834;
    uLINK[5].I(1, 2) = -0.001259;
    uLINK[5].I(2, 0) = 0.028785;
    uLINK[5].I(2, 1) = -0.001259;
    uLINK[5].I(2, 2) = 0.035402;

    strcpy(uLINK[6].name, "RLEG_J4");
    uLINK[6].m  = 0.000000 ;
    uLINK[6].fixed  = 0 ;
    uLINK[6].color  = 1 ;
    uLINK[6].sister  = 0 ;
    uLINK[6].child  = 7 ;
    uLINK[6].fixed  = 0 ;
    uLINK[6].qmin  = -1.570796 ;
    uLINK[6].qmax  = 1.570796 ;
    uLINK[6].qmoy  = -0.174533 ;
    uLINK[6].q  = -0.174533 ;
    uLINK[6].umin  = -2.000000 ;
    uLINK[6].umax  = 2.000000 ;
    uLINK[6].a(0) = 0.000000;
    uLINK[6].a(1) = 1.000000;
    uLINK[6].a(2) = 0.000000;
    uLINK[6].b(0) = 0.000000;
    uLINK[6].b(1) = 0.000000;
    uLINK[6].b(2) = -0.510000;
    uLINK[6].p(0) = 0.046800;
    uLINK[6].p(1) = -0.155000;
    uLINK[6].p(2) = 0.110400;

    uLINK[6].R(0, 0) = 1;
    uLINK[6].R(1, 1) = 1;
    uLINK[6].R(2, 2) = 1;

    uLINK[6].c(0) = 0.000000;
    uLINK[6].c(1) = 0.000000;
    uLINK[6].c(2) = 0.000000;
    uLINK[6].I(0, 0) = 0.000000;
    uLINK[6].I(0, 1) = 0.000000;
    uLINK[6].I(0, 2) = 0.000000;
    uLINK[6].I(1, 0) = 0.000000;
    uLINK[6].I(1, 1) = 0.000000;
    uLINK[6].I(1, 2) = 0.000000;
    uLINK[6].I(2, 0) = 0.000000;
    uLINK[6].I(2, 1) = 0.000000;
    uLINK[6].I(2, 2) = 0.000000;

    strcpy(uLINK[7].name, "RLEG_J5");
    uLINK[7].m  = 1.600000 ;
    uLINK[7].fixed  = 0 ;
    uLINK[7].color  = 1 ;
    uLINK[7].sister  = 0 ;
    uLINK[7].child  = 0 ;
    uLINK[7].fixed  = 0 ;
    uLINK[7].qmin  = -1.570796 ;
    uLINK[7].qmax  = 1.570796 ;
    uLINK[7].qmoy  = 0.000000 ;
    uLINK[7].q  = 0.000000 ;
    uLINK[7].umin  = -2.000000 ;
    uLINK[7].umax  = 2.000000 ;
    uLINK[7].a(0) = 1.000000;
    uLINK[7].a(1) = 0.000000;
    uLINK[7].a(2) = 0.000000;
    uLINK[7].b(0) = 0.000000;
    uLINK[7].b(1) = 0.000000;
    uLINK[7].b(2) = 0.000000;
    uLINK[7].p(0) = 0.046800;
    uLINK[7].p(1) = -0.155000;
    uLINK[7].p(2) = 0.110400;

    uLINK[7].R(0, 0) = 1;
    uLINK[7].R(1, 1) = 1;
    uLINK[7].R(2, 2) = 1;

    uLINK[7].c(0) = 0.085900;
    uLINK[7].c(1) = 0.000000;
    uLINK[7].c(2) = -0.067100;
    uLINK[7].I(0, 0) = 0.003547;
    uLINK[7].I(0, 1) = -0.000014;
    uLINK[7].I(0, 2) = -0.001353;
    uLINK[7].I(1, 0) = -0.000014;
    uLINK[7].I(1, 1) = 0.007363;
    uLINK[7].I(1, 2) = 0.000002;
    uLINK[7].I(2, 0) = -0.001353;
    uLINK[7].I(2, 1) = 0.000002;
    uLINK[7].I(2, 2) = 0.005552;

    strcpy(uLINK[8].name, "LLEG_J0");
    uLINK[8].m  = 0.000000 ;
    uLINK[8].fixed  = 0 ;
    uLINK[8].color  = 2 ;
    uLINK[8].sister  = 0 ;
    uLINK[8].child  = 9 ;
    uLINK[8].fixed  = 0 ;
    uLINK[8].qmin  = -1.570796 ;
    uLINK[8].qmax  = 1.570796 ;
    uLINK[8].qmoy  = 0.000000 ;
    uLINK[8].q  = 0.000000 ;
    uLINK[8].umin  = -2.000000 ;
    uLINK[8].umax  = 2.000000 ;
    uLINK[8].a(0) = 1.000000;
    uLINK[8].a(1) = 0.000000;
    uLINK[8].a(2) = 0.000000;
    uLINK[8].b(0) = 0.046800;
    uLINK[8].b(1) = 0.155000;
    uLINK[8].b(2) = 0.000000;
    uLINK[8].p(0) = 0.046800;
    uLINK[8].p(1) = 0.155000;
    uLINK[8].p(2) = 1.127700;

    uLINK[8].R(0, 0) = 1;
    uLINK[8].R(1, 1) = 1;
    uLINK[8].R(2, 2) = 1;

    uLINK[8].c(0) = -0.005260;
    uLINK[8].c(1) = 0.000000;
    uLINK[8].c(2) = 0.090540;
    uLINK[8].I(0, 0) = 0.067142;
    uLINK[8].I(0, 1) = 0.000000;
    uLINK[8].I(0, 2) = -0.000574;
    uLINK[8].I(1, 0) = 0.000000;
    uLINK[8].I(1, 1) = 0.019827;
    uLINK[8].I(1, 2) = 0.000001;
    uLINK[8].I(2, 0) = -0.000574;
    uLINK[8].I(2, 1) = 0.000001;
    uLINK[8].I(2, 2) = 0.055545;

    strcpy(uLINK[9].name, "LLEG_J1");
    uLINK[9].m  = 0.000000 ;
    uLINK[9].fixed  = 0 ;
    uLINK[9].color  = 2 ;
    uLINK[9].sister  = 0 ;
    uLINK[9].child  = 10 ;
    uLINK[9].fixed  = 0 ;
    uLINK[9].qmin  = -1.570796 ;
    uLINK[9].qmax  = 1.570796 ;
    uLINK[9].qmoy  = -0.174533 ;
    uLINK[9].q  = -0.174533 ;
    uLINK[9].umin  = -2.000000 ;
    uLINK[9].umax  = 2.000000 ;
    uLINK[9].a(0) = 0.000000;
    uLINK[9].a(1) = 1.000000;
    uLINK[9].a(2) = 0.000000;
    uLINK[9].b(0) = 0.000000;
    uLINK[9].b(1) = 0.000000;
    uLINK[9].b(2) = 0.000000;
    uLINK[9].p(0) = 0.046800;
    uLINK[9].p(1) = 0.155000;
    uLINK[9].p(2) = 1.127700;

    uLINK[9].R(0, 0) = 1;
    uLINK[9].R(1, 1) = 1;
    uLINK[9].R(2, 2) = 1;

    uLINK[9].c(0) = 0.006800;
    uLINK[9].c(1) = 0.002500;
    uLINK[9].c(2) = 0.000760;
    uLINK[9].I(0, 0) = 0.000041;
    uLINK[9].I(0, 1) = -0.000002;
    uLINK[9].I(0, 2) = -0.000000;
    uLINK[9].I(1, 0) = -0.000002;
    uLINK[9].I(1, 1) = 0.000034;
    uLINK[9].I(1, 2) = 0.000000;
    uLINK[9].I(2, 0) = -0.000000;
    uLINK[9].I(2, 1) = 0.000000;
    uLINK[9].I(2, 2) = 0.000064;

    strcpy(uLINK[10].name, "LLEG_J2");
    uLINK[10].m  = 8.000000 ;
    uLINK[10].fixed  = 0 ;
    uLINK[10].color  = 2 ;
    uLINK[10].sister  = 0 ;
    uLINK[10].child  = 11 ;
    uLINK[10].fixed  = 0 ;
    uLINK[10].qmin  = -1.570796 ;
    uLINK[10].qmax  = 1.570796 ;
    uLINK[10].qmoy  = 0.000000 ;
    uLINK[10].q  = 0.000000 ;
    uLINK[10].umin  = -2.000000 ;
    uLINK[10].umax  = 2.000000 ;
    uLINK[10].a(0) = 0.000000;
    uLINK[10].a(1) = 0.000000;
    uLINK[10].a(2) = 1.000000;
    uLINK[10].b(0) = 0.000000;
    uLINK[10].b(1) = 0.000000;
    uLINK[10].b(2) = 0.000000;
    uLINK[10].p(0) = 0.046800;
    uLINK[10].p(1) = 0.155000;
    uLINK[10].p(2) = 1.127700;

    uLINK[10].R(0, 0) = 1;
    uLINK[10].R(1, 1) = 1;
    uLINK[10].R(2, 2) = 1;

    uLINK[10].c(0) = 0.032000;
    uLINK[10].c(1) = 0.000000;
    uLINK[10].c(2) = -0.204000;
    uLINK[10].I(0, 0) = 0.086259;
    uLINK[10].I(0, 1) = -0.000098;
    uLINK[10].I(0, 2) = 0.000875;
    uLINK[10].I(1, 0) = -0.000098;
    uLINK[10].I(1, 1) = 0.083201;
    uLINK[10].I(1, 2) = 0.000548;
    uLINK[10].I(2, 0) = 0.000875;
    uLINK[10].I(2, 1) = 0.000548;
    uLINK[10].I(2, 2) = 0.012113;

    strcpy(uLINK[11].name, "LLEG_J3");
    uLINK[11].m  = 10.900000 ;
    uLINK[11].fixed  = 0 ;
    uLINK[11].color  = 0 ;
    uLINK[11].sister  = 0 ;
    uLINK[11].child  = 12 ;
    uLINK[11].fixed  = 0 ;
    uLINK[11].qmin  = -1.570796 ;
    uLINK[11].qmax  = 1.570796 ;
    uLINK[11].qmoy  = 0.349066 ;
    uLINK[11].q  = 0.349066 ;
    uLINK[11].umin  = -2.000000 ;
    uLINK[11].umax  = 2.000000 ;
    uLINK[11].a(0) = 0.000000;
    uLINK[11].a(1) = 1.000000;
    uLINK[11].a(2) = 0.000000;
    uLINK[11].b(0) = 0.000000;
    uLINK[11].b(1) = 0.000000;
    uLINK[11].b(2) = -0.507300;
    uLINK[11].p(0) = 0.046800;
    uLINK[11].p(1) = 0.155000;
    uLINK[11].p(2) = 0.620400;

    uLINK[11].R(0, 0) = 1;
    uLINK[11].R(1, 1) = 1;
    uLINK[11].R(2, 2) = 1;

    uLINK[11].c(0) = 0.000000;
    uLINK[11].c(1) = 0.000000;
    uLINK[11].c(2) = -0.255000;
    uLINK[11].I(0, 0) = 0.193723;
    uLINK[11].I(0, 1) = 0.000010;
    uLINK[11].I(0, 2) = 0.028785;
    uLINK[11].I(1, 0) = 0.000010;
    uLINK[11].I(1, 1) = 0.199834;
    uLINK[11].I(1, 2) = -0.001259;
    uLINK[11].I(2, 0) = 0.028785;
    uLINK[11].I(2, 1) = -0.001259;
    uLINK[11].I(2, 2) = 0.035402;

    strcpy(uLINK[12].name, "LLEG_J4");
    uLINK[12].m  = 0.000000 ;
    uLINK[12].fixed  = 0 ;
    uLINK[12].color  = 1 ;
    uLINK[12].sister  = 0 ;
    uLINK[12].child  = 13 ;
    uLINK[12].fixed  = 0 ;
    uLINK[12].qmin  = -1.570796 ;
    uLINK[12].qmax  = 1.570796 ;
    uLINK[12].qmoy  = -0.174533 ;
    uLINK[12].q  = -0.174533 ;
    uLINK[12].umin  = -2.000000 ;
    uLINK[12].umax  = 2.000000 ;
    uLINK[12].a(0) = 0.000000;
    uLINK[12].a(1) = 1.000000;
    uLINK[12].a(2) = 0.000000;
    uLINK[12].b(0) = 0.000000;
    uLINK[12].b(1) = 0.000000;
    uLINK[12].b(2) = -0.510000;
    uLINK[12].p(0) = 0.046800;
    uLINK[12].p(1) = 0.155000;
    uLINK[12].p(2) = 0.110400;

    uLINK[12].R(0, 0) = 1;
    uLINK[12].R(1, 1) = 1;
    uLINK[12].R(2, 2) = 1;

    uLINK[12].c(0) = 0.000000;
    uLINK[12].c(1) = 0.000000;
    uLINK[12].c(2) = 0.000000;
    uLINK[12].I(0, 0) = 0.000000;
    uLINK[12].I(0, 1) = 0.000000;
    uLINK[12].I(0, 2) = 0.000000;
    uLINK[12].I(1, 0) = 0.000000;
    uLINK[12].I(1, 1) = 0.000000;
    uLINK[12].I(1, 2) = 0.000000;
    uLINK[12].I(2, 0) = 0.000000;
    uLINK[12].I(2, 1) = 0.000000;
    uLINK[12].I(2, 2) = 0.000000;

    strcpy(uLINK[13].name, "LLEG_J5");
    uLINK[13].m  = 1.600000 ;
    uLINK[13].fixed  = 0 ;
    uLINK[13].color  = 1 ;
    uLINK[13].sister  = 0 ;
    uLINK[13].child  = 0 ;
    uLINK[13].fixed  = 0 ;
    uLINK[13].qmin  = -1.570796 ;
    uLINK[13].qmax  = 1.570796 ;
    uLINK[13].qmoy  = 0.000000 ;
    uLINK[13].q  = 0.000000 ;
    uLINK[13].umin  = -2.000000 ;
    uLINK[13].umax  = 2.000000 ;
    uLINK[13].a(0) = 1.000000;
    uLINK[13].a(1) = 0.000000;
    uLINK[13].a(2) = 0.000000;
    uLINK[13].b(0) = 0.000000;
    uLINK[13].b(1) = 0.000000;
    uLINK[13].b(2) = 0.000000;
    uLINK[13].p(0) = 0.046800;
    uLINK[13].p(1) = 0.155000;
    uLINK[13].p(2) = 0.110400;

    uLINK[13].R(0, 0) = 1;
    uLINK[13].R(1, 1) = 1;
    uLINK[13].R(2, 2) = 1;

    uLINK[13].c(0) = 0.085900;
    uLINK[13].c(1) = 0.000000;
    uLINK[13].c(2) = -0.067100;
    uLINK[13].I(0, 0) = 0.003547;
    uLINK[13].I(0, 1) = -0.000014;
    uLINK[13].I(0, 2) = -0.001353;
    uLINK[13].I(1, 0) = -0.000014;
    uLINK[13].I(1, 1) = 0.007363;
    uLINK[13].I(1, 2) = 0.000002;
    uLINK[13].I(2, 0) = -0.001353;
    uLINK[13].I(2, 1) = 0.000002;
    uLINK[13].I(2, 2) = 0.005552;


    uLINK[7].shape[0]=0.280000;
    uLINK[7].shape[1]=0.080000;
    uLINK[7].shape[2]=0.040000;
    uLINK[7].com[0]=0.085300;
    uLINK[7].com[1]=0.000000;
    uLINK[7].com[2]=-0.090000;
    SetupRigidBody(uLINK,7);
    uLINK[7].supportHeight=0.000000;

    uLINK[13].shape[0]=0.280000;
    uLINK[13].shape[1]=0.080000;
    uLINK[13].shape[2]=0.040000;
    uLINK[13].com[0]=0.085300;
    uLINK[13].com[1]=0.000000;
    uLINK[13].com[2]=-0.090000;
    SetupRigidBody(uLINK,13);
    uLINK[13].supportHeight=0.000000;

    uLINK[1].shape[0]=0.100000;
    uLINK[1].shape[1]=0.300000;
    uLINK[1].shape[2]=0.040000;
    uLINK[1].com[0]=0.000000;
    uLINK[1].com[1]=0.000000;
    uLINK[1].com[2]=0.050000;
    SetupRigidBody(uLINK,1);
    uLINK[1].supportHeight=0.000000;


    Status->nb_limb=0;
    Status->limbID = Eigen::VectorXd::Zero(5);
    FindMother(uLINK,Status,1);

    ForwardKinematics(uLINK,1);

    ForwardVelocity(uLINK,1);

    Eigen::Vector3d com;
    CalcCoM(uLINK,com);
    Status->com_old = com;
    

}

