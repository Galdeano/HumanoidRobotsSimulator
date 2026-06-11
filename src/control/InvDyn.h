#ifndef H_InvDyn
#define H_InvDyn

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void InvDyn(SuLINK uLINK[],State *Status,int j,gsl_vector * u);


#endif
