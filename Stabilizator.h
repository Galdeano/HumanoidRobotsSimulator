#ifndef H_Stabilizator
#define H_Stabilizator

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void Stabilizator(SuLINK uLINK[],State *Status,gsl_vector * u,double Dtime,double t);

#endif


