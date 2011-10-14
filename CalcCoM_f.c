#include "uLINK_f.h"

#include "d:\Mb\StLib\Mat.h"

#include "CalcMC_f.h"
#include "TotalMass_f.h"
#include "CalcCoM_f.h"



void CalcCoM_f(Struct_uLINK uLINK[],float *com)
{

    float M;
    MatClrf(com,3,1);
    M = TotalMass_f(uLINK,1);
    CalcMC_f(uLINK,com,1);
    MatMulScalf( com , 1/M , com , 3 , 1 ) ;

}


