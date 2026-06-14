#include <Eigen/Dense>
#include <stdio.h>
#include "uLink.h"
#include "SE3exp.h"
#include "IntegrateEuler.h"
#include "Setup.h"

void IntegrateEuler(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return;
    }

    if (j == 1)
    {
        SE3exp(uLINK, j);
        uLINK[j].vo += Dtime * uLINK[j].dvo;
        uLINK[j].w += Dtime * uLINK[j].dw;
    }
    else
    {
        uLINK[j].q += Dtime * uLINK[j].dq;
        uLINK[j].dq += Dtime * uLINK[j].ddq;
        
        if (uLINK[j].q > uLINK[j].qmax)
        {
            printf("maxq: %d %f %f\n", j, uLINK[j].q, uLINK[j].qmax);
            uLINK[j].q = uLINK[j].qmax;
            uLINK[j].dq = -0.1 * uLINK[j].dq;
        }
        if (uLINK[j].q < uLINK[j].qmin)
        {
            printf("minq: %d %f %f\n", j, uLINK[j].q, uLINK[j].qmin);
            uLINK[j].q = uLINK[j].qmin;
            uLINK[j].dq = -0.1 * uLINK[j].dq;
        }
    }

    IntegrateEuler(uLINK, uLINK[j].sister);
    IntegrateEuler(uLINK, uLINK[j].child);
}
