#include <Eigen/Dense>
#include <stdio.h>
#include "uLink.h"
#include "SE3exp.h"
#include "IntegrateSymplectic.h"
#include "Setup.h"

void IntegrateSymplectic(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return;
    }

    if (j == 1)
    {
        uLINK[j].vo += Dtime * uLINK[j].dvo;
        uLINK[j].w += Dtime * uLINK[j].dw;
        SE3exp(uLINK, j);
    }
    else
    {
        uLINK[j].dq += Dtime * uLINK[j].ddq;
        uLINK[j].q += Dtime * uLINK[j].dq;
        
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

    IntegrateSymplectic(uLINK, uLINK[j].sister);
    IntegrateSymplectic(uLINK, uLINK[j].child);
}
