#ifndef H_ExternalForces
#define H_ExternalForces

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void ExternalForces(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t);


#endif
