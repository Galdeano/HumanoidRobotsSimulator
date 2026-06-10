#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "Spline.h"
#include "OneFootTrajectory.h"


void OneFootTrajectory(SuLINK uLINK[],State *Status, gsl_vector * u, double t)
{
    int n;
//    float kd=100;
//    float kp=100000;
    float kd=100;
    float kp=100000;

    float t1=0.5;
    float t2=2;
    float t3=6;
    float t4=10;

    //Marche static +splines
    gsl_vector * q = gsl_vector_calloc (6);
    gsl_vector * posd = gsl_vector_calloc (3);
    gsl_vector * posg = gsl_vector_calloc (3);

    if (t<t1)
    {
        gsl_vector_set (posd, 0, 0.0);
        gsl_vector_set (posd, 1, 0.0);
        gsl_vector_set (posd, 2, -1.0172);

        gsl_vector_set (posg, 0, 0.0);
        gsl_vector_set (posg, 1, 0.0);
        gsl_vector_set (posg, 2, -1.0172);
    }

    if (t>=t1 && t<t2)
    {
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
        gsl_vector_set (posd, 0, Spline(t,t2,t3,0,-0.09,0,0));
        gsl_vector_set (posd, 1, Spline(t,t2,t3,0,0.29,0,0));
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, Spline(t,t2,t3,0,-0.09,0,0));
        gsl_vector_set (posg, 1, Spline(t,t2,t3,0,0.29,0,0));
        gsl_vector_set (posg, 2, -0.95);
    }

    if (t>=t3 && t<t4)
    {
        gsl_vector_set (posd, 0, -0.09);
        gsl_vector_set (posd, 1, 0.29);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, -0.09);
        gsl_vector_set (posg, 1,  0.29);
        gsl_vector_set (posg, 2, Spline(t,t3,t4,-0.95,-0.9,0,0));
    }


    if (t>=t4)
    {
        gsl_vector_set (posd, 0, -0.09);
        gsl_vector_set (posd, 1, 0.29);
        gsl_vector_set (posd, 2, -0.95);

        gsl_vector_set (posg, 0, -0.09);
        gsl_vector_set (posg, 1,  0.29);
        gsl_vector_set (posg, 2, -0.9);
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

    Status->desired_support=Status->support;


    gsl_vector_free(q);
    gsl_vector_free(posd);
    gsl_vector_free(posg);



    for(n=2; n<14; n++)
    {
        gsl_vector_set (u,(n+4),uLINK[n].u_joint);
    }

}



