#ifndef H_OneFootTrajectory
#define H_OneFootTrajectory

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void OneFootTrajectory(SuLINK uLINK[],State *Status, gsl_vector * u, double t);


#endif
