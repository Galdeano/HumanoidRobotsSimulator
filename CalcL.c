#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcL.h"
#include "Cross.h"




void CalcL( SuLINK uLINK[], gsl_vector * L, int j)
{
    static gsl_vector * tmp;
    static gsl_vector * tmp2;
    static gsl_matrix * tmp3;
    static gsl_matrix * tmp4;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        tmp = gsl_vector_calloc (3);
        tmp2 = gsl_vector_calloc (3);
        tmp3 = gsl_matrix_calloc (3, 3);
        tmp4 = gsl_matrix_calloc (3, 3);

        init_tmp=0;
    }

    if (j == 0)
    {
        return;
    }
    else
    {
//        gsl_vector * tmp = gsl_vector_calloc (3);
//        gsl_vector * tmp2 = gsl_vector_calloc (3);
//        gsl_matrix * tmp3 = gsl_matrix_calloc (3, 3);
//        gsl_matrix * tmp4 = gsl_matrix_calloc (3, 3);

        /*        c1 = uLINK(j).R * uLINK(j).c;*/
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, tmp);
        /*        c  = uLINK(j).p + c1;*/
        gsl_vector_memcpy (tmp2, tmp);
        gsl_vector_add (tmp2, uLINK[j].p);
        /*        P = uLINK(j).m * (uLINK(j).v + cross(c1, uLINK(j).w));*/
        Cross(tmp,uLINK[j].w,0);
        gsl_vector_add (tmp, uLINK[j].v);
        gsl_vector_scale (tmp, uLINK[j].m);

        /*        L = cross(c, P) + uLINK(j).R * uLINK(j).I * uLINK(j).R' * uLINK(j).w;*/
        Cross(tmp2,tmp,1);
        gsl_vector_add (L, tmp);
        gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].I, 0.0, tmp3);
        gsl_blas_dgemm (CblasNoTrans, CblasTrans, 1.0, tmp3, uLINK[j].R, 0.0, tmp4);
        gsl_blas_dgemv(CblasNoTrans, 1.0, tmp4, uLINK[j].w, 0.0, tmp);
        gsl_vector_add (L, tmp);

//        gsl_vector_free(tmp);
//        gsl_vector_free(tmp2);
//        gsl_matrix_free(tmp3);
//        gsl_matrix_free(tmp4);

        /*        L = L + calcL(uLINK(j).sister) + calcL(uLINK(j).child);*/
        CalcL(uLINK,L,uLINK[j].child);
        CalcL(uLINK,L,uLINK[j].sister);

    }
}
