#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcMC.h"


void CalcMC( SuLINK uLINK[], gsl_vector * com, int j)
{
    if (j == 0)
    {
        return;
    }
    else
    {
        gsl_vector * tmp = gsl_vector_calloc (3);

/*        mc = uLINK(j).m * (uLINK(j).p + uLINK(j).R * uLINK(j).c);*/
        gsl_blas_dgemv(CblasNoTrans, 1.0, uLINK[j].R, uLINK[j].c, 0.0, tmp);
        gsl_vector_add (tmp, uLINK[j].p);
        gsl_vector_scale (tmp, uLINK[j].m);
        gsl_vector_add (com, tmp);

/*        mc = mc + calcMC(uLINK(j).sister) + calcMC(uLINK(j).child);*/
        CalcMC(uLINK,com,uLINK[j].child);
        CalcMC(uLINK,com,uLINK[j].sister);

        gsl_vector_free(tmp);
    }
}

