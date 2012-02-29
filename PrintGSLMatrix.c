#include <gsl/gsl_matrix.h>
#include "PrintGSLMatrix.h"

void PrintGSLMatrix(gsl_matrix * M)
{
    int i,j;
    for (i=0; i<M->size1; i++)
    {
        for (j=0; j<M->size2; j++)
        {
            printf("%4.6f ",gsl_matrix_get(M, i,j));
        }
        printf("\n");
    }
    printf("\n");
}


void PrintGSLMatrixTranspose(gsl_matrix * M)
{
    int i,j;
    for (i=0; i<M->size2; i++)
    {
        for (j=0; j<M->size1; j++)
        {
            printf("%4.6f ",gsl_matrix_get(M, j,i));
        }
        printf("\n");
    }
    printf("\n");
}


void PrintGSLVector(gsl_vector * V)
{
    int i;
    for (i=0; i<V->size; i++)
    {
       printf("%4.6f ",gsl_vector_get(V, i));
    }
    printf("\n");
}


