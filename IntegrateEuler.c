#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "SE3exp.h"
#include "IntegrateEuler.h"

#include "Setup.h"

void IntegrateEuler(SuLINK uLINK[],int j )
{
    if (j == 0)
    {
        return;
    }

    if (j == 1)
    {
        SE3exp(uLINK,j);

        gsl_vector * tmp = gsl_vector_calloc (3);
        gsl_vector_memcpy (tmp, uLINK[j].dvo);
        gsl_vector_scale (tmp, Dtime);
        gsl_vector_add (uLINK[j].vo, tmp);
        gsl_vector_memcpy (tmp, uLINK[j].dw);
        gsl_vector_scale (tmp, Dtime);
        gsl_vector_add (uLINK[j].w, tmp);
        gsl_vector_free(tmp);

        //uLINK[j].vo = uLINK[j].vo + Dtime * uLINK[j].dvo;
        //uLINK[j].w  = uLINK[j].w  + Dtime * uLINK[j].dw;
    }
    else
    {
        uLINK[j].q  = uLINK[j].q  + Dtime * uLINK[j].dq;
        uLINK[j].dq = uLINK[j].dq + Dtime * uLINK[j].ddq;

    }

    IntegrateEuler(uLINK,uLINK[j].sister);
    IntegrateEuler(uLINK,uLINK[j].child);

}

