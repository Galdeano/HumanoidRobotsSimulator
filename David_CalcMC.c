#include "David_uLINK.h"

#include "d:\Mb\StLib\Mat.h"

#include "David_CalcMC.h"


void David_CalcMC( Struct_uLINK uLINK[], float *com, int j)
{
    if (j == 0)
    {
        return;
    }
    else
    {

        float tmp[3];


        MatMulf( tmp, uLINK[j].R, uLINK[j].c, 3, 3, 1) ;
        MatAddf( tmp, tmp, uLINK[j].p, 3 , 1) ;
        MatMulScalf( tmp, uLINK[j].m, tmp, 3, 1) ;
        MatAddf( com, com, tmp, 3, 1) ;

        David_CalcMC(uLINK,com,uLINK[j].child);
        David_CalcMC(uLINK,com,uLINK[j].sister);

    }
}

