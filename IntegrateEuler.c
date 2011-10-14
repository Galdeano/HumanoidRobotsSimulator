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
        //[uLINK(j).p, uLINK(j).R] = SE3exp(j, Dtime);
        SE3exp(uLINK,j);

//printf(" j: %d",j);
//printf(" c: \n");
//printf("%8.4f \n",gsl_vector_get (uLINK[j].p,0));
//printf("%8.4f \n",gsl_vector_get (uLINK[j].p,1));
//printf("%8.4f \n \n",gsl_vector_get (uLINK[j].p,2));
//
//    printf(" R: \n");
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 0,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 0,1));
//printf("%8.4f \n",gsl_matrix_get (uLINK[j].R, 0,2));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 1,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 1,1));
//printf("%8.4f \n",gsl_matrix_get (uLINK[j].R, 1,2));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 2,0));
//printf("%8.4f ",gsl_matrix_get (uLINK[j].R, 2,1));
//printf("%8.4f \n \n",gsl_matrix_get (uLINK[j].R, 2,2));



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
//        printf(" j: %d \n",j);
//        //printf("%8.4f \n",uLINK[j].q);
//        printf("%8.4f \n",uLINK[j].dq);
    }

    IntegrateEuler(uLINK,uLINK[j].sister);
    IntegrateEuler(uLINK,uLINK[j].child);

}

