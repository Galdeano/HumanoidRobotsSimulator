#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcCoP.h"


double CalcCoP( SuLINK uLINK[], gsl_vector * CoP, int j)
{
    static gsl_vector * v;
    static gsl_vector * ftmp;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        v = gsl_vector_calloc (3);
        ftmp = gsl_vector_calloc (3);

        init_tmp=0;
    }
    if (j == 0)
    {
        return 0.0;
    }
    else
    {
        double f=0;
        if (uLINK[j].isPolygon==1)
        {
            int i;
            for (i = 0; i < 8; ++i)
            {
                if (gsl_vector_get(uLINK[j].isContact,i)==1)
                {
                    //gsl_vector * v = gsl_vector_calloc (3);
                    //gsl_vector * ftmp = gsl_vector_calloc (3);
                    gsl_matrix_get_col (v, uLINK[j].posContact, i);
                    gsl_vector_set (v, 2, 0);
                    gsl_matrix_get_col (ftmp, uLINK[j].forContact, i);
                    gsl_vector_scale (v, gsl_vector_get (ftmp,2));
                    gsl_vector_add (CoP, v);
                    f=f+gsl_vector_get (ftmp,2);
                    //gsl_vector_free(ftmp);
                    //gsl_vector_free(v);
                }
            }
        }

        f=f+CalcCoP(uLINK,CoP,uLINK[j].child);
        f=f+CalcCoP(uLINK,CoP,uLINK[j].sister);
        return f;

    }

}
