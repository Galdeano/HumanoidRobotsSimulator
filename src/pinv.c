//#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_permutation.h>
//#include <gsl/gsl_math.h>
#include "pinv.h"


int pinv( gsl_matrix * invA, gsl_matrix * A)
{

    int n_fil;
    int n_col;
    n_fil =A->size1;
    n_col =A->size2;

    int i = 0;


#ifndef GSL_RANGE_CHECK_OFF
    if ((A->size1 !=invA->size2) || (A->size2 !=invA->size1))
    {
        GSL_ERROR ("invalid length", GSL_EBADLEN);
    }
#endif


    gsl_matrix * gA_t = gsl_matrix_alloc (n_col, n_fil);
    gsl_matrix_transpose_memcpy (gA_t, A);					// Computing the transpose of gA

    gsl_matrix * U = gsl_matrix_alloc (n_col, n_fil);
    gsl_matrix * V= gsl_matrix_alloc (n_fil, n_fil);
    gsl_vector * S = gsl_vector_alloc (n_fil);


    // Computing the SVD of the transpose of A
    // The matrix 'gA_t' will contain 'U' after the function is called
    gsl_vector * work = gsl_vector_alloc (n_fil);
    gsl_linalg_SV_decomp (gA_t, V, S, work);
    gsl_vector_free(work);

    gsl_matrix_memcpy (U, gA_t);


    //Inverting S//
    //----------------------------------------------------------
    // Matrix 'S' is diagonal, so it is contained in a vector.
    // We operate to convert the vector 'S' into the matrix 'Sp'.
    //Then we invert 'Sp' to 'Spu'
    //----------------------------------------------------------
    gsl_matrix * Sp = gsl_matrix_alloc (n_fil, n_fil);
    gsl_matrix_set_zero (Sp);
    for (i = 0; i < n_fil; i++)
        gsl_matrix_set (Sp, i, i, gsl_vector_get(S, i));	// Vector 'S' to matrix 'Sp'

    gsl_permutation * p = gsl_permutation_alloc (n_fil);
    int signum;
    gsl_linalg_LU_decomp (Sp, p, &signum);				// Computing the LU decomposition

    // Compute the inverse like in the MATLAB script

    gsl_matrix * SI = gsl_matrix_calloc (n_fil, n_fil);

    for (i = 0; i < n_fil; i++)
    {
        if (gsl_vector_get (S, i) > 0.0000000001)
            gsl_matrix_set (SI, i, i, 1.0 / gsl_vector_get (S, i));
    }

    gsl_matrix * VT = gsl_matrix_alloc (n_fil, n_fil);
    gsl_matrix_transpose_memcpy (VT, V);					// Tranpose of V


    //THE PSEUDOINVERSE//
    //----------------------------------------------------------
    //Computation of the pseudoinverse of trans(A) as pinv(A) = U·inv(S).trans(V)   with trans(A) = U.S.trans(V)
    //----------------------------------------------------------
    gsl_matrix * SIpVT = gsl_matrix_alloc (n_fil, n_fil);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,				// Calculating  inv(S).trans(V)
                    1.0, SI, VT,
                    0.0, SIpVT);


    //gsl_matrix * pinv = gsl_matrix_alloc (n_col, n_fil);	// Calculating  U·inv(S).trans(V)
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, U, SIpVT,
                    0.0, invA);

    //end THE PSEUDOINVERSE//


    gsl_matrix_free(gA_t);
    gsl_matrix_free(U);
    gsl_matrix_free(V);
    gsl_vector_free(S);

    gsl_matrix_free(Sp);
    gsl_permutation_free(p);
    gsl_matrix_free(SI);
    gsl_matrix_free(VT);
    gsl_matrix_free(SIpVT);

    return GSL_SUCCESS;

}

