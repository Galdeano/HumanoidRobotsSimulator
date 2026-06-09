/*---------------------------------------------------------------------------------------------------|
    Compute the nullspace of a matrix A [mxn] using a QR decompostion (transpose(A) = QR)
    The (n-m) columns of Q span the nullspace of A.                                                           |
----------------------------------------------------------------------------------------------------*/

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_math.h>
//#include <gsl/gsl_vector.h>

#include "nullspace.h"

void nullspace (gsl_matrix *A, gsl_matrix *Anull)
{
    gsl_matrix *At = gsl_matrix_alloc (A->size2, A->size1);
    gsl_matrix_transpose_memcpy (At, A);                         // Computes At = transpose(A)

    int n_row;
    int n_col;
    n_row = At->size1;
    n_col = At->size2;

    gsl_matrix *QR = gsl_matrix_alloc (n_row, n_col);
    gsl_matrix *Q = gsl_matrix_alloc (n_row, n_row);
    gsl_matrix *R = gsl_matrix_alloc (n_row, n_col);

    gsl_matrix_memcpy(QR, At);					                 // Copies transpose(A) into a QR matrix

    gsl_vector *tau = gsl_vector_alloc (gsl_min(n_row, n_col));
    gsl_linalg_QR_decomp (QR, tau);                              // Computes the encoded QR decomposition
    gsl_linalg_QR_unpack (QR, tau, Q, R);                        // Unpacks the encoded QR decomposition into the matrices Q and R

    gsl_vector *v = gsl_vector_alloc(n_row);

    int i;
    for (i = 0; i<(n_row-n_col); i++)
    {
        gsl_matrix_get_col (v, Q, (n_row-i-1));
        gsl_matrix_set_col (Anull, i, v);
    };

    // Test nullspace validity
    //----------------------------------------------------------
//    gsl_matrix *ApAnull = gsl_matrix_alloc  (n_col, (n_row-n_col));
//    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,				     // Calculating  A.Anull
//                    1.0, A, Anull,
//                    0.0, ApAnull);
//    int j;
//    for (i = 0; i < 6; i++)
//        for (j = 0; j < 2; j++)
//            printf ("ApAnull(%d,%d) = %g\n", i, j,gsl_matrix_get (ApAnull, i, j));
    //----------------------------------------------------------

    gsl_matrix_free (QR);
    gsl_matrix_free (Q);
    gsl_matrix_free (R);
    gsl_vector_free (v);
    gsl_vector_free (tau);

}
