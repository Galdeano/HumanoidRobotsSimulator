#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include "InverseSherpaKinematics.h"


void InverseSherpaKinematics(gsl_vector * q,gsl_vector * pos)
{
//   double Hy=0.155*2;
//   double Hx=0.0468;
    double Lc=0.5073;
    double Lt=0.510;
//    double Lp=0.1104;

    double x=gsl_vector_get (pos,0);
    double y=gsl_vector_get (pos,1);
    double z=gsl_vector_get (pos,2);

    gsl_vector_set (q, 0,atan2(y,-z));
    gsl_vector_set (q, 5,-atan2(y,-z));

    double l,q4,q2;

    l=gsl_hypot(y,z);

    q4=atan2(sqrt(1-gsl_pow_2(((gsl_pow_2(l)+gsl_pow_2(x)-gsl_pow_2(Lc)-gsl_pow_2(Lt))/(2*Lc*Lt)))) , (gsl_pow_2(l)+gsl_pow_2(x)-gsl_pow_2(Lc)-gsl_pow_2(Lt))/(2*Lc*Lt));
    gsl_vector_set (q, 3,q4);

    q2=-atan2(x,l)-atan2(Lt*sin(q4),Lc+Lt*cos(q4));
    gsl_vector_set (q, 1,q2);

    gsl_vector_set (q, 4,-q4-q2);
}
