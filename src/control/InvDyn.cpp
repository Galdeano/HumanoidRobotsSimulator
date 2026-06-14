#include <Eigen/Dense>
#include "uLink.h"
#include "ForwardAllKinematics.h"
#include "InverseDynamics.h"
#include "InvDyn.h"

void InvDyn(SuLINK uLINK[], State *Status, int j, Eigen::VectorXd & u)
{
    int n;
    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    uLINK[1].dvo.setZero();
    uLINK[1].dw.setZero();

    if (j >= 1 && j <= 3)
    {
        uLINK[1].dvo(j-1) = 1.0;
    }
    if (j >= 4 && j <= 6)
    {
        uLINK[1].dw(j-4) = 1.0;
    }

    for (n=1; n<((Status->dof)-6+2-1); n++)
    {
        if (n == j-6)
            uLINK[n+1].ddq = 1.0;
        else
            uLINK[n+1].ddq = 0.0;
    }

    ForwardAllKinematics(uLINK, 1);

    InverseDynamics(uLINK, Status, 1, f, t);

    for (n=0; n<3; n++)
    {
        u(n) = f(n);
    }
    for (n=0; n<3; n++)
    {
        u(n+3) = t(n);
    }
    for (n=0; n<((Status->dof)-6); n++)
    {
        u(n+6) = uLINK[n+2].u;
    }
}
