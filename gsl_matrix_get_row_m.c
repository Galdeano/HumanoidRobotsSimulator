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
        GSL_ERROR ("invalid length row", GSL_EBADLEN);
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
        GSL_ERROR ("invalid length column", GSL_EBADLEN);
    }
#endif
}




int gsl_matrix_get_part_m(gsl_matrix * matrix_out, gsl_matrix * matrix_in, int s_row, int e_row, int s_column, int e_column)
{
    int i,j;
#ifndef GSL_RANGE_CHECK_OFF
    if (((s_column-e_column) == matrix_out->size2)&&((s_row-e_row) == matrix_out->size1))
    {
#endif
        for (i=s_row; i<e_row; i++)
        {
            for (j=s_column; j<e_column; j++)
            {
                gsl_matrix_set(matrix_out,i-s_row,j-s_column,gsl_matrix_get(matrix_in, i,j));
            }
        }
        return GSL_SUCCESS;
#ifndef GSL_RANGE_CHECK_OFF
    }
    else
    {   printf("%d %d %d %d ",s_row-e_row,s_column-e_column,matrix_out->size1,matrix_out->size2);
        PrintGSLMatrix(matrix_in);
        GSL_ERROR ("invalid length part", GSL_EBADLEN);
    }
#endif
}

int gsl_matrix_get_part2_m(gsl_matrix * matrix_out, gsl_matrix * matrix_in, int s_row, int e_row, int s_column, int e_column)
{
    int i,j;
#ifndef GSL_RANGE_CHECK_OFF
    if (((e_column-s_column) == matrix_out->size2)&&((e_row-s_row) == matrix_out->size1))
    {
#endif
//printf("%d ",1000);
        for (i=e_row; i<s_row; i++)
        {//printf("i %d ",i);
            for (j=e_column; j<s_column; j++)
            {
            //printf("%d ",i*j+j);
                gsl_matrix_set(matrix_out,i-e_row,j-e_column,gsl_matrix_get(matrix_in, i,j));
            }
        }//PrintGSLMatrix(matrix_in);
        return GSL_SUCCESS;
#ifndef GSL_RANGE_CHECK_OFF
    }
    else
    {
        //printf("%d %d %d %d ",e_row-s_row,e_column-s_column,matrix_out->size1,matrix_out->size2);
        //PrintGSLMatrix(matrix_in);
        GSL_ERROR ("invalid length part", GSL_EBADLEN);
    }
#endif
}
