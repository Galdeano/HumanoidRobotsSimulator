#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "Spline.h"
#include "StaticTrajectory.h"




void PDStaticTrajectory(SuLINK uLINK[],State *Status, gsl_vector * qd, double d_time, double t)
{
    int n;
//    float kd=100;
//    float kp=100000;

    //float kd=100;
    //float kp=10000;

    float t1=2;
    float t2=5;
    float t3=15;
    float t4=25;
    float t5=35;
    float t6=45;
    float t7=55;
    float t8=65;
//    float t9=75;
//	float t10=85;
//	float t11=95;
//	float t12=105;


//    float t1=2;
//    float t2=5;
//    float t3=10;
//    float t4=15;
//    float t5=20;
//    float t6=25;
//    float t7=30;
//    float t8=35;
//    float t9=40;
//    float t10=45;
//    float t11=50;
//    float t12=55;


//    float t1=0.5;
//    float t2=2;
//    float t3=6;
//    float t4=10;
//    float t5=14;
//    float t6=18;
//    float t7=22;
//    float t8=26;

//    float t1=0.5;
//    float t2=2;
//    float t3=5;
//    float t4=8;
//    float t5=11;
//    float t6=14;
//    float t7=17;
//    float t8=20;


    float com_y =0.29;
    //float com_y =0.29;
    float adjust_x =-0.03;
    //float steep_x =0.05;


//    while(t>=t8)
//    {
//        t -= t8-t4;
//    }


    //Marche static +splines
    gsl_vector * q = gsl_vector_calloc (6);
    gsl_vector * posd = gsl_vector_calloc (3);
    gsl_vector * posg = gsl_vector_calloc (3);

    if (t<t1)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, 0.0);
        gsl_vector_set (posd, 1, 0.0);
        gsl_vector_set (posd, 2, -1.0172);

        gsl_vector_set (posg, 0, 0.0);
        gsl_vector_set (posg, 1, 0.0);
        gsl_vector_set (posg, 2, -1.0172);

        Status->distribution_y=0.5;
    }

    if (t>=t1 && t<t2)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, 0.0);
        gsl_vector_set (posd, 1, 0.0);
        //gsl_vector_set (posd, 2, Spline(t,t1,t2,-1.0172,-0.95,0,0));
        gsl_vector_set (posd, 2, C2Spline(t,t1,(t1+t2)/2,t2,-1.0172,-1.000,-0.95,0,0));

        gsl_vector_set (posg, 0, 0.0);
        gsl_vector_set (posg, 1, 0.0);
        //gsl_vector_set (posg, 2, Spline(t,t1,t2,-1.0172,-0.95,0,0));
        gsl_vector_set (posg, 2, C2Spline(t,t1,(t1+t2)/2,t2,-1.0172,-1.000,-0.95,0,0));

        Status->distribution_y=0.5;
    }

    if (t>=t2 && t<t3)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t2,t3,0,-0.05,0,0));
        gsl_vector_set (posd, 1, Spline(t,t2,t3,0,com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t2,t3,0,-0.05,0,0));
        gsl_vector_set (posg, 1, Spline(t,t2,t3,0,com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);

        Status->distribution_y=Spline(t,t2,t3,0.5,0.0,0,0);
    }

    if (t>=t3 && t<t4)
    {
        Status->desired_support=1;
        gsl_vector_set (posd, 0, -0.05+Spline(t,t3,t4,0.0,-adjust_x,0,0));
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t3,t4,-0.05,0.05,0,0)+Spline(t,t3,t4,0.0,-adjust_x,0,0));
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, C2Spline(t,t3,(t3+t4)/2,t4,-0.95,-0.90,-0.95,0,0));

        Status->distribution_y=0.0;

    }


    if (t>=t4 && t<t5)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t4,t5,-0.05,-0.15,0,0)+Spline(t,t4,t5,-adjust_x,adjust_x,0,0));
        gsl_vector_set (posd, 1, Spline(t,t4,t5,com_y,-com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t4,t5,0.05,-0.05,0,0)+Spline(t,t4,t5,-adjust_x,adjust_x,0,0));
        gsl_vector_set (posg, 1, Spline(t,t4,t5,com_y,-com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);

        Status->distribution_y=Spline(t,t4,t5,0.0,1.0,0,0);
    }


    if (t>=t5 && t<t6)
    {
        Status->desired_support=2;
        gsl_vector_set (posd, 0, Spline(t,t5,t6,-0.15,0.05,0,0)+Spline(t,t5,t6,adjust_x,-adjust_x,0,0));
        gsl_vector_set (posd, 1, -com_y);
        gsl_vector_set (posd, 2, C2Spline(t,t5,(t5+t6)/2,t6,-0.95,-0.90,-0.95,0,0));

        gsl_vector_set (posg, 0, -0.05+Spline(t,t5,t6,adjust_x,-adjust_x,0,0));
        gsl_vector_set (posg, 1,  -com_y);
        gsl_vector_set (posg, 2, -0.95);

        Status->distribution_y=1.0;
    }


    if (t>=t6 && t<t7)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t6,t7,0.05,-0.05,0,0)+Spline(t,t6,t7,-adjust_x,adjust_x,0,0));
        gsl_vector_set (posd, 1, Spline(t,t6,t7,-com_y,com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t6,t7,-0.05,-0.15,0,0)+Spline(t,t6,t7,-adjust_x,adjust_x,0,0));
        gsl_vector_set (posg, 1,  Spline(t,t6,t7,-com_y,com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);

        Status->distribution_y=Spline(t,t6,t7,1.0,0.0,0,0);
    }


    if (t>=t7 && t<t8)
    {
        Status->desired_support=1;
        gsl_vector_set (posd, 0, -0.05+Spline(t,t7,t8,adjust_x,-adjust_x,0,0));
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t7,t8,-0.15,0.05,0,0)+Spline(t,t7,t8,adjust_x,-adjust_x,0,0));
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, C2Spline(t,t7,(t7+t8)/2,t8,-0.95,-0.90,-0.95,0,0));

        Status->distribution_y=0.0;
    }


    if (t>=t8 )
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, -0.05-adjust_x);
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, 0.05-adjust_x);
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, -0.95);

        Status->distribution_y=0.0;
    }








    InverseSherpaKinematics(q, posd);
    for(n=0; n<6; n++)
    {
        gsl_vector_set (qd,n+6,gsl_vector_get (q,n)) ;
    }


    InverseSherpaKinematics(q, posg);
    for(n=0; n<6; n++)
    {
        gsl_vector_set (qd,n+12,gsl_vector_get (q,n)) ;
    }


    //Status->desired_support=Status->support;

    gsl_vector_free(q);
    gsl_vector_free(posd);
    gsl_vector_free(posg);


}




void PStaticTrajectory(SuLINK uLINK[],State *Status, gsl_vector * u, double t)
{
    int n;
//    float kd=100;
//    float kp=100000;
    float kd=100;
    float kp=10000;
    //float kd=100;
    //float kp=10000;

//    float t1=0.5;
//    float t2=2;
//    float t3=6;
//    float t4=10;
//    float t5=14;
//    float t6=18;
//    float t7=22;
//    float t8=26;
//    float t9=30;
//    float t10=34;

    float t1=0.5;
    float t2=2;
    float t3=5;
    float t4=8;
    float t5=11;
    float t6=14;
    float t7=17;
    float t8=20;

//    float t1=0.5;
//    float t2=2;
//    float t3=3;
//    float t4=4;
//    float t5=5;
//    float t6=6;
//    float t7=7;
//    float t8=8;


    float com_y =0.28;

    while(t>=20)
    {
        t -= t8-t4;
    }


    //Marche static +splines
    gsl_vector * q = gsl_vector_calloc (6);
    gsl_vector * posd = gsl_vector_calloc (3);
    gsl_vector * posg = gsl_vector_calloc (3);

    if (t<t1)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, 0.0);
        gsl_vector_set (posd, 1, 0.0);
        gsl_vector_set (posd, 2, -1.0172);

        gsl_vector_set (posg, 0, 0.0);
        gsl_vector_set (posg, 1, 0.0);
        gsl_vector_set (posg, 2, -1.0172);
    }

    if (t>=t1 && t<t2)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, 0.0);
        gsl_vector_set (posd, 1, 0.0);
        //gsl_vector_set (posd, 2, Spline(t,t1,t2,-1.0172,-0.95,0,0));
        gsl_vector_set (posd, 2, C2Spline(t,t1,(t1+t2)/2,t2,-1.0172,-1.000,-0.95,0,0));

        gsl_vector_set (posg, 0, 0.0);
        gsl_vector_set (posg, 1, 0.0);
        //gsl_vector_set (posg, 2, Spline(t,t1,t2,-1.0172,-0.95,0,0));
        gsl_vector_set (posg, 2, C2Spline(t,t1,(t1+t2)/2,t2,-1.0172,-1.000,-0.95,0,0));
    }

    if (t>=t2 && t<t3)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t2,t3,0,-0.05,0,0));
        gsl_vector_set (posd, 1, Spline(t,t2,t3,0,com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t2,t3,0,-0.05,0,0));
        gsl_vector_set (posg, 1, Spline(t,t2,t3,0,com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);
    }

    if (t>=t3 && t<t4)
    {
        Status->desired_support=1;
        gsl_vector_set (posd, 0, -0.05);
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t3,t4,-0.05,0.05,0,0));
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, C2Spline(t,t3,(t3+t4)/2,t4,-0.95,-0.90,-0.95,0,0));

    }


    if (t>=t4 && t<t5)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t4,t5,-0.05,-0.15,0,0));
        gsl_vector_set (posd, 1, Spline(t,t4,t5,com_y,-com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t4,t5,0.05,-0.05,0,0));
        gsl_vector_set (posg, 1, Spline(t,t4,t5,com_y,-com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);
    }


    if (t>=t5 && t<t6)
    {
        Status->desired_support=2;
        gsl_vector_set (posd, 0, Spline(t,t5,t6,-0.15,0.05,0,0));
        gsl_vector_set (posd, 1, -com_y);
        gsl_vector_set (posd, 2, C2Spline(t,t5,(t5+t6)/2,t6,-0.95,-0.90,-0.95,0,0));

        gsl_vector_set (posg, 0, -0.05);
        gsl_vector_set (posg, 1,  -com_y);
        gsl_vector_set (posg, 2, -0.95);
    }


    if (t>=t6 && t<t7)
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, Spline(t,t6,t7,0.05,-0.05,0,0));
        gsl_vector_set (posd, 1, Spline(t,t6,t7,-com_y,com_y,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t6,t7,-0.05,-0.15,0,0));
        gsl_vector_set (posg, 1,  Spline(t,t6,t7,-com_y,com_y,0,0));
        gsl_vector_set (posg, 2, -0.95);
    }

    if (t>=t7 && t<t8)
    {
        Status->desired_support=1;
        gsl_vector_set (posd, 0, -0.05);
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t7,t8,-0.15,0.05,0,0));
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, C2Spline(t,t7,(t7+t8)/2,t8,-0.95,-0.90,-0.95,0,0));
    }


    if (t>=t8 )
    {
        Status->desired_support=3;
        gsl_vector_set (posd, 0, -0.05);
        gsl_vector_set (posd, 1, com_y);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, 0.05);
        gsl_vector_set (posg, 1,  com_y);
        gsl_vector_set (posg, 2, -0.95);
    }




    InverseSherpaKinematics(q, posd);
    for(n=2; n<8; n++)
    {
        uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
    }

    InverseSherpaKinematics(q, posg);
    for(n=8; n<14; n++)
    {
        uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
    }


    //Status->desired_support=Status->support;

    gsl_vector_free(q);
    gsl_vector_free(posd);
    gsl_vector_free(posg);



    for(n=2; n<14; n++)
    {
        gsl_vector_set (u,(n+4),uLINK[n].u_joint);
    }

}


