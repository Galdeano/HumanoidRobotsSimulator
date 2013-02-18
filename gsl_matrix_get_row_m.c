#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "gsl_matrix_get_row_m.h"

int gsl_matrix_get_row_m(gsl_matrix * matrix_out, gsl_matrix * matrix_in, int row)
{
    int i;
#ifndef GSL_RANGE_CHECK_OFF
    if (matrix_in->size2 == matrix_out->size2)
    {
#endif
        for (i=0; i<matrix_in->size2; i++)
        {
            gsl_matrix_set(matrix_out,0,i,gsl_matrix_get(matrix_in, row,i));
        }
        return GSL_SUCCESS;
#ifndef GSL_RANGE_CHECK_OFF
    }
    else
    {
        GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
#endif
}

int gsl_matrix_get_column_m(gsl_matrix * matrix_out, gsl_matrix * matrix_in, int column)
{
    int i;
#ifndef GSL_RANGE_CHECK_OFF
    if (matrix_in->size1 == matrix_out->size1)
    {
#endif
        for (i=0; i<matrix_in->size1; i++)
        {
            gsl_matrix_set(matrix_out,i,0,gsl_matrix_get(matrix_in, i,column));
        }
        return GSL_SUCCESS;
#ifndef GSL_RANGE_CHECK_OFF
    }
    else
    {
        GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
#endif
}


