#include <Eigen/Dense>
#include "uLink.h"
#include "CalcP.h"

void CalcP(SuLINK uLINK[], Eigen::Vector3d & P, int j)
{
    if (j == 0)
    {
        return;
    }
    else
    {
        Eigen::Vector3d c1 = uLINK[j].R * uLINK[j].c;
        P += uLINK[j].m * (uLINK[j].v + c1.cross(uLINK[j].w));

        CalcP(uLINK, P, uLINK[j].child);
        CalcP(uLINK, P, uLINK[j].sister);
    }
}
