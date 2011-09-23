#ifndef H_StaticTrajectory
#define H_StaticTrajectory

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


void PDStaticTrajectory(SuLINK uLINK[],State *Status, gsl_vector * u, double Dtime, double t);

void PStaticTrajectory(SuLINK uLINK[],State *Status, gsl_vector * u, double t);


#endif
