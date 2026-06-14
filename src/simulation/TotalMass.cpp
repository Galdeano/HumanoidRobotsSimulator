#include "uLink.h"
#include "TotalMass.h"

double TotalMass(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return 0.0;
    }
    else
    {
        return uLINK[j].m + TotalMass(uLINK, uLINK[j].sister) + TotalMass(uLINK, uLINK[j].child);
    }
}
