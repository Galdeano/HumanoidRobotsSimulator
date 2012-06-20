#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "ForwardVelocity.h"
#include "Cross.h"



void ForwardVelocity(SuLINK uLINK[], int j)
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
    if (j != 1)
    {

        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[uLINK[j].mother].R, uLINK[uLINK[j].mother].b, 0.0, uLINK[j].v);
        Cross(uLINK[uLINK[j].mother].w,uLINK[j].v,1);
        gsl_vector_add (uLINK[j].v, uLINK[uLINK[j].mother].v);
        //uLINK[j].v = uLINK[uLINK[j].mother].v + cross(uLINK[uLINK[j].mother].w, uLINK[uLINK[j].mother].R * uLINK[uLINK[j].mother].b);
//        gsl_vector * tmp = gsl_vector_calloc (3);
        gsl_vector_memcpy (tmp, uLINK[j].a);
        gsl_vector_scale (tmp, uLINK[j].dq);
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, tmp, 0.0, uLINK[j].w);
        gsl_vector_add (uLINK[j].w, uLINK[uLINK[j].mother].w);
//        gsl_vector_free(tmp);
//printf("%d: %8.4f ",j,gsl_vector_get (uLINK[j].w,0));
//printf("%8.4f ",gsl_vector_get (uLINK[j].w,1));
//printf("%8.4f \n",gsl_vector_get (uLINK[j].w,2));
        //uLINK[j].w = uLINK[uLINK[j].mother].w + uLINK[j].R * (uLINK[j].a * uLINK[j].dq);
    }
    ForwardVelocity(uLINK,uLINK[j].sister);
    ForwardVelocity(uLINK,uLINK[j].child);


}


