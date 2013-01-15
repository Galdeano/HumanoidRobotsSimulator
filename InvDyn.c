#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLink.h"
#include "ForwardAllKinematics.h"
#include "InverseDynamics.h"
#include "InvDyn.h"

#include "Setup.h"


void InvDyn(SuLINK uLINK[],State *Status,int j,gsl_vector * u)
{
    int n;
    static gsl_vector * f;
    static gsl_vector * t;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        f = gsl_vector_calloc (3);
        t = gsl_vector_calloc (3);
        init_tmp=0;
    }

    gsl_vector_set_zero(uLINK[1].dvo);
    //gsl_vector_set (uLINK[1].dvo, 2,9.81+gsl_vector_get (uLINK[1].dvo,2));
    gsl_vector_set_zero(uLINK[1].dw);
//uLINK(1).dvo = uLINK(1).dvo + [0 0 G]';
//uLINK(1).dvo = [0 0 0]';
//uLINK(1).dw  = [0 0 0]';

    if (j >= 1 && j <= 3)
    {
        gsl_vector_set (uLINK[1].dvo, j-1, 1);
    }
    if (j >= 4 && j <= 6)
    {
        gsl_vector_set (uLINK[1].dw, j-4, 1);
    }
//if j >= 1 & j <= 3
//    uLINK(1).dvo(j) = 1;
//elseif j >= 4 & j <= 6
//    uLINK(1).dw(j-3) = 1;
//end
//

    for (n=1; n<((Status->ddl)-6+2-1); n++)
    {
        if (n == j-6)
            uLINK[n+1].ddq = 1;
        else
            uLINK[n+1].ddq = 0;
    }
//for n=1:length(uLINK)-1
//    if n == j-6
//        uLINK(n+1).ddq = 1;
//    else
//        uLINK(n+1).ddq = 0;
//    end
//end
//

    ForwardAllKinematics(uLINK,1);
//ForwardAllKinematics(1);

//    gsl_vector * f = gsl_vector_calloc (3);
//    gsl_vector * t = gsl_vector_calloc (3);

    InverseDynamics(uLINK,Status,1,f,t);
//[f,t] = InverseDynamics(1);

    for (n=0; n<3; n++)
    {
        gsl_vector_set (u, n,gsl_vector_get (f,n));
    }
    for (n=0; n<3; n++)
    {
        gsl_vector_set (u, n+3,gsl_vector_get (t,n));
    }
    for (n=0; n<((Status->ddl)-6); n++)
    {
        gsl_vector_set (u, n+6,uLINK[n+2].u);
    }
//u = [f',t',uLINK(2:end).u]';
//    gsl_vector_free(f);
//    gsl_vector_free(t);
}

