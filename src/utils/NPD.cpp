
#include <cmath>
#include "NPD.h"

double npd(double e,double alpha,double sigma)
{
    if (e>sigma)
    {
        return pow(fabs(e),alpha-1.0);
    }
    else
    {
        return pow(sigma,alpha-1.0);
    }
}
