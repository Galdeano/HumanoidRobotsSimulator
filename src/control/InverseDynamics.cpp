#include <Eigen/Dense>
#include "uLink.h"
#include "Hat.h"
#include "InverseDynamics.h"

void InverseDynamics(SuLINK uLINK[], State *Status, int j, Eigen::Vector3d & f, Eigen::Vector3d & t)
{
    if (j == 0)
    {
        f.setZero();
        t.setZero();
        return;
    }

    Eigen::Vector3d c = uLINK[j].R * uLINK[j].c + uLINK[j].p;   // center of mass
    Eigen::Matrix3d I = uLINK[j].R * uLINK[j].I * uLINK[j].R.transpose();  // inertia in world frame

    Eigen::Matrix3d c_hat;
    Hat(c_hat, c);
    I += uLINK[j].m * c_hat * c_hat.transpose();

    Eigen::Vector3d P = uLINK[j].m * (uLINK[j].vo + uLINK[j].w.cross(c));   // linear momentum
    Eigen::Vector3d L = uLINK[j].m * c.cross(uLINK[j].vo) + I * uLINK[j].w; // angular momentum

    f = uLINK[j].m * (uLINK[j].dvo + uLINK[j].dw.cross(c)) + uLINK[j].w.cross(P);
    t = uLINK[j].m * c.cross(uLINK[j].dvo) + I * uLINK[j].dw + uLINK[j].vo.cross(P) + uLINK[j].w.cross(L);

    // from child link
    Eigen::Vector3d ftmp = Eigen::Vector3d::Zero();
    Eigen::Vector3d ttmp = Eigen::Vector3d::Zero();
    InverseDynamics(uLINK, Status, uLINK[j].child, ftmp, ttmp);
    f += ftmp;
    t += ttmp;

    if (j != 1)
    {
        uLINK[j].u = uLINK[j].hv.dot(f) + uLINK[j].hw.dot(t);  // joint driving force
    }

    // return force to mother, with sisters
    ftmp.setZero();
    ttmp.setZero();
    InverseDynamics(uLINK, Status, uLINK[j].sister, ftmp, ttmp);
    f += ftmp;
    t += ttmp;
}
