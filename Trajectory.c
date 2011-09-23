#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "uLINK.h"
#include "InverseSherpaKinematics.h"
#include "CalcCoM.h"
#include "Spline.h"
#include "Trajectory.h"



void Trajectory(SuLINK uLINK[], gsl_vector * u, double t)
{
    int n;
    float kd=100;
    float kp=100000;

    for(n=2; n<14; n++)
    {
        //uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(0-uLINK[n].q) ;
    }







    //Marche static +splines
    if (1)
    {
        gsl_vector * q = gsl_vector_calloc (6);
        gsl_vector * pos = gsl_vector_calloc (3);

        if (t<0.3)
        {
            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0172);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0172);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
        }

        if (t>=0.3 && t<1)
        {
            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, Spline(t,0.3,1.0,-1.0172,-0.95,0,0));
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, Spline(t,0.3,1.0,-1.0172,-0.95,0,0));
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
        }

        int pas=0;

        if (t>=1 && t<3 && pas==0)
        {
//            gsl_vector * com = gsl_vector_calloc (3);
//            CalcCoM(uLINK,com);
//
//            gsl_vector * PosPied = gsl_vector_calloc (3);
//            double a[] = { 0.0853, 0.0, -0.11};
//            gsl_vector_view A = gsl_vector_view_array (a, 3);
//            gsl_blas_dgemv (CblasNoTrans, 1.0, uLINK[7].R, &A.vector, 0.0, PosPied);
//            gsl_vector_add (PosPied, uLINK[7].p);
//            gsl_vector_sub (PosPied, com);
//            gsl_vector_set (pos, 0, 0.5*gsl_vector_get (PosPied,0));
//            gsl_vector_set (pos, 1, 0.5*gsl_vector_get (PosPied,1));

            gsl_vector_set (pos, 0, Spline(t,1.0,3.0,0,-0.09,0,0));
            gsl_vector_set (pos, 1, Spline(t,1.0,3.0,0,0.32,0,0));
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, Spline(t,1.0,3.0,0,-0.09,0,0));
            gsl_vector_set (pos, 1, Spline(t,1.0,3.0,0,0.32,0,0));
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
//            gsl_vector_free(com);
        }

        if (t>=3 && t<5 && pas==0)
        {
//            gsl_vector * com = gsl_vector_calloc (3);
//            CalcCoM(uLINK,com);
//
//            gsl_vector * PosPied = gsl_vector_calloc (3);
//            double a[] = { 0.0853, 0.0, -0.11};
//            gsl_vector_view A = gsl_vector_view_array (a, 3);
//            gsl_blas_dgemv (CblasNoTrans, 1.0, uLINK[7].R, &A.vector, 0.0, PosPied);
//            gsl_vector_add (PosPied, uLINK[7].p);
//            gsl_vector_sub (PosPied, com);
//            gsl_vector_set (pos, 0, 0.5*gsl_vector_get (PosPied,0));
//            gsl_vector_set (pos, 1, 0.5*gsl_vector_get (PosPied,1));

            gsl_vector_set (pos, 0, -0.09);
            gsl_vector_set (pos, 1, 0.32);
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, -0.09);
            gsl_vector_set (pos, 1,  0.32);
            gsl_vector_set (pos, 2, Spline(t,3.0,5.0,-0.95,-0.9,0,0));
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
//            gsl_vector_free(com);
        }


        if (t>=5 && pas==0)
        {

            gsl_vector_set (pos, 0, -0.09);
            gsl_vector_set (pos, 1, 0.32);
            gsl_vector_set (pos, 2, -0.95);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, -0.09);
            gsl_vector_set (pos, 1,  0.32);
            gsl_vector_set (pos, 2, -0.9);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
//            gsl_vector_free(com);
        }

        gsl_vector_free(q);
        gsl_vector_free(pos);
    }












    //Marche static
    if (0)
    {
        gsl_vector * q = gsl_vector_calloc (6);
        gsl_vector * pos = gsl_vector_calloc (3);

        if (t<0.5)
        {
            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0172);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0172);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
        }

        if (t>=0.5 && t<1)
        {
            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0170+(t-0.5)*0.2);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.0);
            gsl_vector_set (pos, 2, -1.0170+(t-0.5)*0.2);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
        }

        int pas=0;

        if (t>=1 && t<1.7 && pas==0)
        {
//            gsl_vector * com = gsl_vector_calloc (3);
//            CalcCoM(uLINK,com);
//
//            gsl_vector * PosPied = gsl_vector_calloc (3);
//            double a[] = { 0.0853, 0.0, -0.11};
//            gsl_vector_view A = gsl_vector_view_array (a, 3);
//            gsl_blas_dgemv (CblasNoTrans, 1.0, uLINK[7].R, &A.vector, 0.0, PosPied);
//            gsl_vector_add (PosPied, uLINK[7].p);
//            gsl_vector_sub (PosPied, com);
//            gsl_vector_set (pos, 0, 0.5*gsl_vector_get (PosPied,0));
//            gsl_vector_set (pos, 1, 0.5*gsl_vector_get (PosPied,1));

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.15);
            gsl_vector_set (pos, 2, -0.917);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.15);
            gsl_vector_set (pos, 2, -0.910);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
//            gsl_vector_free(com);
        }

        if (t>=1.7 && pas==0)
        {
//            gsl_vector * com = gsl_vector_calloc (3);
//            CalcCoM(uLINK,com);
//
//            gsl_vector * PosPied = gsl_vector_calloc (3);
//            double a[] = { 0.0853, 0.0, -0.11};
//            gsl_vector_view A = gsl_vector_view_array (a, 3);
//            gsl_blas_dgemv (CblasNoTrans, 1.0, uLINK[7].R, &A.vector, 0.0, PosPied);
//            gsl_vector_add (PosPied, uLINK[7].p);
//            gsl_vector_sub (PosPied, com);
//            gsl_vector_set (pos, 0, 0.5*gsl_vector_get (PosPied,0));
//            gsl_vector_set (pos, 1, 0.5*gsl_vector_get (PosPied,1));

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.26);
            gsl_vector_set (pos, 2, -0.917);
            InverseSherpaKinematics(q, pos);
            for(n=2; n<8; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
            }

            gsl_vector_set (pos, 0, 0.0);
            gsl_vector_set (pos, 1, 0.26);
            gsl_vector_set (pos, 2, -0.80);
            InverseSherpaKinematics(q, pos);
            for(n=8; n<14; n++)
            {
                uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
            }
//            gsl_vector_free(com);
        }

        gsl_vector_free(q);
        gsl_vector_free(pos);
    }

    //demo iverse kinematics
    if (0)
    {
        gsl_vector * q = gsl_vector_calloc (6);
        gsl_vector * pos = gsl_vector_calloc (3);

        gsl_vector_set (pos, 0, 0.1*sin(t*2));
        gsl_vector_set (pos, 1, 0.0);
        gsl_vector_set (pos, 2, -0.9);
        InverseSherpaKinematics(q, pos);
        for(n=2; n<8; n++)
        {
            uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2)-uLINK[n].q) ;
        }

        gsl_vector_set (pos, 0, 0.1*sin(t*2));
        gsl_vector_set (pos, 1, 0.0);
        gsl_vector_set (pos, 2, -0.9);
        InverseSherpaKinematics(q, pos);
        for(n=8; n<14; n++)
        {
            uLINK[n].u_joint = kd*(0-uLINK[n].dq)+kp*(gsl_vector_get (q,n-2-6)-uLINK[n].q) ;
        }

        gsl_vector_free(q);
        gsl_vector_free(pos);
    }

    //static: 1=active
    if (0)
    {
        uLINK[3].u_joint = kd*(0-uLINK[3].dq)+kp*(-0.1-uLINK[3].q) ;
        uLINK[5].u_joint = kd*(0-uLINK[5].dq)+kp*(0.2-uLINK[5].q) ;
        uLINK[6].u_joint = kd*(0-uLINK[6].dq)+kp*(-0.1-uLINK[6].q) ;
        uLINK[9].u_joint = kd*(0-uLINK[9].dq)+kp*(-0.1-uLINK[9].q) ;
        uLINK[11].u_joint = kd*(0-uLINK[11].dq)+kp*(0.2-uLINK[11].q) ;
        uLINK[12].u_joint = kd*(0-uLINK[12].dq)+kp*(-0.1-uLINK[12].q) ;
    }

    if (0)
    {
        double position = (sin(M_PI*t*4)*(-0.10) -0.10);
        double velocity = (M_PI*4*cos(M_PI*t*4)*(-0.10));

        uLINK[6].u_joint = kd*(velocity-uLINK[6].dq)+kp*(position-uLINK[6].q) ;
        uLINK[12].u_joint = kd*(velocity-uLINK[12].dq)+kp*(position-uLINK[12].q) ;
    }

    if (0)
    {
        double position = (sin(M_PI*t)*(-0.18) -0.00);
        double velocity = (M_PI*cos(M_PI*t)*(-0.18));

        uLINK[6].u_joint = kd*(velocity-uLINK[6].dq)+kp*(position-uLINK[6].q) ;
        uLINK[12].u_joint = kd*(velocity-uLINK[12].dq)+kp*(position-uLINK[12].q) ;
    }



    for(n=2; n<14; n++)
    {
        gsl_vector_set (u,(n+4),uLINK[n].u_joint);
    }

}



