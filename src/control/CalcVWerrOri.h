#ifndef H_CalcVWerrOri
#define H_CalcVWerrOri

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void CalcVWerrOri( SuLINK uLINK[], gsl_vector * err, gsl_vector * p, gsl_matrix * R, gsl_vector * idx);

#endif


//    gsl_vector * err = gsl_vector_calloc (6);
//    gsl_vector * p = gsl_vector_calloc (3);
//    gsl_matrix * R = gsl_matrix_calloc (3,3);
//    gsl_matrix_set_identity(R);
//
//    //gsl_vector_set (p, 0, 0.0);
//
//    CalcVWerrOri(uLINK, err, p, R,idx);
//    PrintGSLVector(err);
