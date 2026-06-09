#ifndef H_InverseDynamics
#define H_InverseDynamics

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void InverseDynamics(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t);

#endif
