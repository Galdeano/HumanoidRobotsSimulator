#ifndef H_pinv
#define H_pinv

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


int pinv( gsl_matrix * invM, gsl_matrix * M);

#endif

//
//gsl_matrix * A = gsl_matrix_alloc (4, 4);
//gsl_matrix * invA = gsl_matrix_alloc (4, 4);
//	double mat[4][4] =  {{ 50,4.5, -23,  12},		// Example Rank-deficient matrix
//			             {  1,  2,   3,   4},
//		             	 {  1,  2,   3,   4},
//	    	             {  2,  4,   6,   8}};
//	for (i = 0; i < 4; i++)
//		for (j = 0; j < 4; j++)
//	   		gsl_matrix_set (A, i, j, mat[i][j]);
//
//pinv( invA, A);
//
//for (i=0; i<invA->size1; i++)
//{
//    for (j=0; j<invA->size2; j++)
//    {
//        printf("%4.6f ",gsl_matrix_get(invA, i,j));
//    }
//    printf("\n");
//}
//printf("\n");
//

