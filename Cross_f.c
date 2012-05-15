
#include "Mat.h"

#include "Cross_f.h"


int Cross_f( float *a, float *b, int i)
{
    float cross[3];
    cross[0]=a[1]*b[2]-a[2]*b[1];
    cross[1]=a[2]*b[0]-a[0]*b[2];
    cross[2]=a[0]*b[1]-a[1]*b[0];

    if (i==0)
    {
        MatCpyf( a, cross, 3, 1) ;
    }
    else
    {
        MatCpyf( b, cross, 3, 1) ;
    }

    return 0;
}

