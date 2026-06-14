#include <Eigen/Dense>
#include "uLink.h"
#include "ForwardVelocity.h"

void ForwardVelocity(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return;
    }

    if (j != 1)
    {
        int mom = uLINK[j].mother;
        uLINK[j].v = uLINK[mom].v + uLINK[mom].w.cross(uLINK[mom].R * uLINK[mom].b);
        uLINK[j].w = uLINK[mom].w + uLINK[j].R * (uLINK[j].a * uLINK[j].dq);
    }

    ForwardVelocity(uLINK, uLINK[j].sister);
    ForwardVelocity(uLINK, uLINK[j].child);
}
