#ifndef H_ForwardDynamics
#define H_ForwardDynamics

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void ForwardDynamics(SuLINK uLINK[],State *Status,double Dtime,long t);


#endif
