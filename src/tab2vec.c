#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "tab2vec.h"


void tab2vec(gsl_vector *vec, double *tab)
{
    int i;
    for (i=0; i<vec->size; i++)
    {
       gsl_vector_set(vec, i, tab[i]);
    }

}
