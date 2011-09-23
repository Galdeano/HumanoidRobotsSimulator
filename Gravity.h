#ifndef H_Gravity
#define H_Gravity

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void Gravity(SuLINK uLINK[],State *Status,int j,gsl_vector * f,gsl_vector * t);


#endif
