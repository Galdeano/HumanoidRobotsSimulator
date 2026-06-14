#include <Eigen/Dense>
#include "uLink.h"
#include "CalcL.h"

void CalcL(SuLINK uLINK[], Eigen::Vector3d & L, int j)
{
    if (j == 0)
    {
        return;
    }
    else
    {
        Eigen::Vector3d c1 = uLINK[j].R * uLINK[j].c;
        Eigen::Vector3d c  = uLINK[j].p + c1;
        Eigen::Vector3d P  = uLINK[j].m * (uLINK[j].v + c1.cross(uLINK[j].w));

        L += c.cross(P) + uLINK[j].R * uLINK[j].I * uLINK[j].R.transpose() * uLINK[j].w;

        CalcL(uLINK, L, uLINK[j].child);
        CalcL(uLINK, L, uLINK[j].sister);
    }
}
