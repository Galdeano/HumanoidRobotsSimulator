#ifndef H_Rodrigues
#define H_Rodrigues

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void Rodrigues(gsl_matrix * rot,gsl_vector * w,double dt);

#endif
