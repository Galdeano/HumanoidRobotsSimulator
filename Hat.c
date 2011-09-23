#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "Hat.h"


void Hat(gsl_matrix * hat,gsl_vector * vec)
{
    gsl_matrix_set_zero(hat);

    gsl_matrix_set (hat, 0, 1, -gsl_vector_get (vec,2));
    gsl_matrix_set (hat, 0, 2, gsl_vector_get (vec,1));
    gsl_matrix_set (hat, 1, 0, gsl_vector_get (vec,2));
    gsl_matrix_set (hat, 1, 2, -gsl_vector_get (vec,0));
    gsl_matrix_set (hat, 2, 0, -gsl_vector_get (vec,1));
    gsl_matrix_set (hat, 2, 1, gsl_vector_get (vec,0));
}


