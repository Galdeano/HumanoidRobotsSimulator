#include "uLINK_f.h"

#include "Mat.h"

#include "CalcMC_f.h"
#include "TotalMass_f.h"
#include "CalcCoM_f.h"



void CalcCoM_f(Struct_uLINK uLINK[],double *com)
{
    static double M;
    static int init_tmp=1;
    if (init_tmp==1)
    {
        M = TotalMass_f(uLINK,1);
        init_tmp=0;
    }
    //double M;
    MatClrf(com,3,1);
    //M = TotalMass_f(uLINK,1);
    CalcMC_f(uLINK,com,1);
    MatMulScalf( com , 1/M , com , 3 , 1 ) ;

}


