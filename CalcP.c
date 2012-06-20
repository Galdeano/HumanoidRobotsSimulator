#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcP.h"
#include "Cross.h"


void CalcP( SuLINK uLINK[], gsl_vector * P, int j)
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
    else
    {
//        gsl_vector * tmp = gsl_vector_calloc (3);
        /*         c1 = uLINK(j).R * uLINK(j).c;
         *         P = uLINK(j).m * (uLINK(j).v + cross(c1, uLINK(j).w) );
         */
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, tmp);
        Cross(tmp,uLINK[j].w,0);
        gsl_vector_add (tmp, uLINK[j].v);
        gsl_vector_scale (tmp, uLINK[j].m);
        gsl_vector_add (P, tmp);
//        gsl_vector_free(tmp);
        /*        P = P + calcP(uLINK(j).sister) + calcP(uLINK(j).child);*/
        CalcP(uLINK,P,uLINK[j].child);
        CalcP(uLINK,P,uLINK[j].sister);

    }
}

