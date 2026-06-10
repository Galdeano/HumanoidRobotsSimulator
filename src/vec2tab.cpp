#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "vec2tab.h"


void vec2tab(double *tab, gsl_vector *vec)
{
    int i;
    for (i=0; i<vec->size; i++)
    {
       tab[i]=gsl_vector_get(vec, i);
    }

}

