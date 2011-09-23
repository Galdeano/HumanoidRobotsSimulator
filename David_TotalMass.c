#include "David_uLINK.h"
#include "David_TotalMass.h"




float David_TotalMass(Struct_uLINK uLINK[],int j)
{
    float m;
    if (j == 0)
    {
        m = 0;
    }
    else
    {
        //printf("%d : %f \n",j,uLINK[j].m);
        m = uLINK[j].m + David_TotalMass(uLINK,uLINK[j].sister) + David_TotalMass(uLINK,uLINK[j].child);
        //printf("%f \n",m);
    }
    return m;
}


