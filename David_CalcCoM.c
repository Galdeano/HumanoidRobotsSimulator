#include "David_uLINK.h"

#include "d:\Mb\StLib\Mat.h"

#include "David_CalcMC.h"
#include "David_TotalMass.h"
#include "David_CalcCoM.h"



void David_CalcCoM(Struct_uLINK uLINK[],float *com)
{

    float M;
    MatClrf(com,3,1);
    M = David_TotalMass(uLINK,1);
    David_CalcMC(uLINK,com,1);
    MatMulScalf( com , 1/M , com , 3 , 1 ) ;

}


