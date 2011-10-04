#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "uLINK.h"
#include "CalcMC.h"
#include "TotalMass.h"
#include "CalcCoM.h"




void CalcCoM(SuLINK uLINK[],gsl_vector * com)
{
    /*
    //function com = calcCoM()
    //global uLINK
    //M  = TotalMass(1);
    //MC = calcMC(1);
    //com = MC / M;
    */

    float M;
    gsl_vector_set_zero(com);
    M = TotalMass(uLINK,1);
    CalcMC(uLINK,com,1);
    gsl_vector_scale (com, 1/M);

}


