#include <math.h>
#include "InverseSherpaKinematics_f.h"


void InverseSherpaKinematics_f(double *qd,double *pos)
{
    /* //   double Hy=0.155*2;
     * //   double Hx=0.0468;
     * //    double Lp=0.1104;
     */
    static const double Lc=0.5073f;
    static const double Lt=0.510f;


    double x=pos[0];
    double y=pos[1];
    double z=pos[2];


    double l=(double)hypot(y,z);


    qd[0]=(double)atan2(y,-z);
    qd[5]=-qd[0];

    qd[3]=(double)atan2(sqrt(1-pow((l*l+x*x-Lc*Lc-Lt*Lt)/(2*Lc*Lt),2.0)) , (l*l+x*x-Lc*Lc-Lt*Lt)/(2*Lc*Lt));

    qd[1]=-(double)atan2(x,l)-(double)atan2(Lt*sin(qd[3]),Lc+Lt*cos(qd[3]));

    qd[4]=-qd[3]-qd[1];
    qd[2]=0;
}
