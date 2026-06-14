#include <Eigen/Dense>
#include "uLink.h"
#include "ForwardKinematics.h"
#include "Rodrigues.h"

void ForwardKinematics(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return;
    }

    if (j != 1)
    {
        int mom = uLINK[j].mother;
        uLINK[j].p = uLINK[mom].R * uLINK[j].b + uLINK[mom].p;

        Eigen::Matrix3d rot;
        Rodrigues(rot, uLINK[j].a, uLINK[j].q);
        uLINK[j].R = uLINK[mom].R * rot;
    }

    ForwardKinematics(uLINK, uLINK[j].sister);
    ForwardKinematics(uLINK, uLINK[j].child);
}
