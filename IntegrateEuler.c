#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "SE3exp.h"
#include "IntegrateEuler.h"

#include "Setup.h"

void IntegrateEuler(SuLINK uLINK[],int j )
{
    static gsl_vector * tmp;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        tmp = gsl_vector_calloc (3);
        init_tmp=0;
    }

    if (j == 0)
    {
        return;
    }

    if (j == 1)
    {
        SE3exp(uLINK,j);

//        gsl_vector * tmp = gsl_vector_calloc (3);
        gsl_vector_memcpy (tmp, uLINK[j].dvo);
        gsl_vector_scale (tmp, Dtime);
        gsl_vector_add (uLINK[j].vo, tmp);
        gsl_vector_memcpy (tmp, uLINK[j].dw);
        gsl_vector_scale (tmp, Dtime);
        gsl_vector_add (uLINK[j].w, tmp);
//        gsl_vector_free(tmp);

        //uLINK[j].vo = uLINK[j].vo + Dtime * uLINK[j].dvo;
        //uLINK[j].w  = uLINK[j].w  + Dtime * uLINK[j].dw;
    }
    else
    {
        uLINK[j].q  = uLINK[j].q  + Dtime * uLINK[j].dq;
        uLINK[j].dq = uLINK[j].dq + Dtime * uLINK[j].ddq;
        if (uLINK[j].q>uLINK[j].qmax)
        {
            printf("maxq: %d %f %f\n",j,uLINK[j].q,uLINK[j].qmax);
            uLINK[j].q=uLINK[j].qmax;
            uLINK[j].dq=-0.1*uLINK[j].dq;
        }
        if (uLINK[j].q<uLINK[j].qmin)
        {
            printf("minq: %d %f %f\n",j,uLINK[j].q,uLINK[j].qmin);
            uLINK[j].q=uLINK[j].qmin;
            uLINK[j].dq=-0.1*uLINK[j].dq;
            //printf("minq: %d\n",j);

        }
    }

    IntegrateEuler(uLINK,uLINK[j].sister);
    IntegrateEuler(uLINK,uLINK[j].child);

}

