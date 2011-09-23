#ifndef H_Spline
#define H_Spline

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>


double Spline(double x,double x0,double x1,double p0,double p1,double v0,double v1);

double C2Spline(double x,double x0,double x1,double x2,double p0,double p1,double p2,double v0,double v2);

#endif
