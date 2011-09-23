#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "Cross.h"


int Cross(gsl_vector * a, gsl_vector * b, int i)
{
    gsl_vector * cross = gsl_vector_alloc (3);
    gsl_vector_set (cross,0,gsl_vector_get (a,1)*gsl_vector_get (b,2)-gsl_vector_get (a,2)*gsl_vector_get (b,1));
    gsl_vector_set (cross,1,gsl_vector_get (a,2)*gsl_vector_get (b,0)-gsl_vector_get (a,0)*gsl_vector_get (b,2));
    gsl_vector_set (cross,2,gsl_vector_get (a,0)*gsl_vector_get (b,1)-gsl_vector_get (a,1)*gsl_vector_get (b,0));
    if (i==0)
    {
        gsl_vector_memcpy (a, cross);
    }
    else
    {
        gsl_vector_memcpy (b, cross);
    }

    gsl_vector_free(cross);
    return 0;
}

