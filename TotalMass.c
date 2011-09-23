#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include "uLINK.h"
#include "TotalMass.h"




float TotalMass(SuLINK uLINK[],int j)
{
    float m;
    if (j == 0)
    {
        m = 0;
    }
    else
    {
        //printf("%d : %f \n",j,uLINK[j].m);
        m = uLINK[j].m + TotalMass(uLINK,uLINK[j].sister) + TotalMass(uLINK,uLINK[j].child);
        //printf("%f \n",m);
    }
    return m;
}



