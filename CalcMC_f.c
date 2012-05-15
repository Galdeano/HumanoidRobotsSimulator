#include "uLINK_f.h"

#include "Mat.h"

#include "CalcMC_f.h"


void CalcMC_f( Struct_uLINK uLINK[], float *com, int j)
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

        CalcMC_f(uLINK,com,uLINK[j].child);
        CalcMC_f(uLINK,com,uLINK[j].sister);

    }
}

