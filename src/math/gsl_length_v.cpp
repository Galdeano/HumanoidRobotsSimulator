
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "gsl_length_v.h"

int gsl_length_v(gsl_vector * vector)
{
    return vector->size;
}


