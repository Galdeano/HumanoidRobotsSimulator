#include <Eigen/Dense>
#include "uLink.h"
#include "Cross.h"
#include "CalcCoM.h"
#include "TotalMass.h"
#include "Gravity.h"
#include "Setup.h"

void Gravity(SuLINK uLINK[], State *Status, int j, Eigen::Vector3d & f, Eigen::Vector3d & t)
{
    f.setZero();
    t.setZero();

    if (j == 0)
    {
        return;
    }

    Eigen::Vector3d ftmp = Eigen::Vector3d::Zero();
    Eigen::Vector3d ttmp = Eigen::Vector3d::Zero();

    Eigen::Vector3d w_c = uLINK[j].R * uLINK[j].c + uLINK[j].p;
    ftmp(2) = -9.81 * uLINK[j].m;
    ttmp = w_c.cross(ftmp);

    f += ftmp;
    t += ttmp;

    Eigen::Vector3d fchild = Eigen::Vector3d::Zero();
    Eigen::Vector3d tchild = Eigen::Vector3d::Zero();
    Gravity(uLINK, Status, uLINK[j].child, fchild, tchild);
    f += fchild;
    t += tchild;

    if (j != 1)
    {
        uLINK[j].ug = uLINK[j].hv.dot(f) + uLINK[j].hw.dot(t);
    }

    Eigen::Vector3d fsister = Eigen::Vector3d::Zero();
    Eigen::Vector3d tsister = Eigen::Vector3d::Zero();
    Gravity(uLINK, Status, uLINK[j].sister, fsister, tsister);
    f += fsister;
    t += tsister;
}
