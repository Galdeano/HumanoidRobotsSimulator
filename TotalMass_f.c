#include "uLINK_f.h"
#include "TotalMass_f.h"




float TotalMass_f(Struct_uLINK uLINK[],int j)
{
    float m;
    if (j == 0)
    {
        m = 0;
    }
    else
    {
        //printf("%d : %f \n",j,uLINK[j].m);
        m = uLINK[j].m + TotalMass_f(uLINK,uLINK[j].sister) + TotalMass_f(uLINK,uLINK[j].child);
        //printf("%f \n",m);
    }
    return m;
}


