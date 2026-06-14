#include <Eigen/Dense>
#include "uLink.h"
#include "CalcMC.h"

void CalcMC(SuLINK uLINK[], Eigen::Vector3d & com, int j)
{
    if (j == 0)
    {
        return;
    }
    else
    {
        Eigen::Vector3d tmp = uLINK[j].R * uLINK[j].c + uLINK[j].p;
        tmp *= uLINK[j].m;
        com += tmp;

        CalcMC(uLINK, com, uLINK[j].child);
        CalcMC(uLINK, com, uLINK[j].sister);
    }
}
