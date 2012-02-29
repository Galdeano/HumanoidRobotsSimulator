#ifndef H_PrintGSLMatrix
#define H_PrintGSLMatrix

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void PrintGSLMatrix(gsl_matrix * M);

void PrintGSLMatrixTranspose(gsl_matrix * M);

void PrintGSLVector(gsl_vector * V);

#endif
