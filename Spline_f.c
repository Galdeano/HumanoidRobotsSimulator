#include "Spline_f.h"


float Spline_f(float x,float x0,float x1,float p0,float p1,float v0,float v1)
{

    float t= (x-x0)/(x1-x0);

    float h1 =  2*t*t*t - 3*t*t + 1;          // calculate basis function 1
    float h2 = -2*t*t*t + 3*t*t;              // calculate basis function 2
    float h3 =    t*t*t - 2*t*t + t;          // calculate basis function 3
    float h4 =    t*t*t -   t*t;              // calculate basis function 4
    return  h1*p0 +                    // multiply and sum all funtions
            h2*p1 +                    // together to build the interpolated
            h3*v0*(x1-x0) +            // point along the curve.
            h4*v1*(x1-x0);

}

float C2Spline_f(float x,float x0,float x1,float x2,float p0,float p1,float p2,float v0,float v2)
{

    float v1 = ((-6*p0-2*(x1-x0)*v0+6*p1)/((x1-x0)*(x1-x0))+(-6*p1+6*p2-2*(x2-x1)*v2)/((x2-x1)*(x2-x1)))/(4/(x1-x0)+4/(x2-x1));

    if (x<=x1)
    {
        return Spline_f(x,x0,x1,p0,p1,v0,v1);
    }

    if (x>x1)
    {
        return Spline_f(x,x1,x2,p1,p2,v1,v2);
    }
    return 0;
}

