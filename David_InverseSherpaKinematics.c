#include <math.h>
#include "David_InverseSherpaKinematics.h"


void David_InverseSherpaKinematics(float *qd,float *pos)
{
    /* //   float Hy=0.155*2;
     * //   float Hx=0.0468;
     * //    float Lp=0.1104;
     */
    static const float Lc=0.5073f;
    static const float Lt=0.510f;


    float x=pos[0];
    float y=pos[1];
    float z=pos[2];


    float l=(float)hypot(y,z);


    qd[0]=(float)atan2(y,-z);
    qd[5]=-qd[0];

    qd[3]=(float)atan2(sqrt(1-pow((l*l+x*x-Lc*Lc-Lt*Lt)/(2*Lc*Lt),2.0)) , (l*l+x*x-Lc*Lc-Lt*Lt)/(2*Lc*Lt));

    qd[1]=-(float)atan2(x,l)-(float)atan2(Lt*sin(qd[3]),Lc+Lt*cos(qd[3]));

    qd[4]=-qd[3]-qd[1];
    qd[2]=0;
}
