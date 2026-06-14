#include <Eigen/Dense>
#include "uLink.h"
#include "CalcCoM.h"
#include "Stabilizator.h"

void Stabilizator(SuLINK uLINK[], State *Status, Eigen::VectorXd & stab, double t)
{
    if (Status->desired_support == 0)
    {
        Status->integral_R = 0.0;
        Status->integral_L = 0.0;

        Eigen::Vector3d com;
        CalcCoM(uLINK, com);
        Status->com_old = com;
    }

    if (Status->desired_support == 3)
    {
        Eigen::Vector3d com;
        CalcCoM(uLINK, com);
        Status->com_old = com;
    }

    if (Status->desired_support == 1)
    {
        Eigen::Vector3d com;
        CalcCoM(uLINK, com);

        stab(10) = - 100.0 * (Status->FootCenter_R(0) - com(0)) + 20.0 * (com(0) - Status->com_old(0)) / Dtime;
        stab(11) = + 200.0 * (Status->FootCenter_R(1) - com(1)) - 20.0 * (com(1) - Status->com_old(1)) / Dtime;

        Status->com_old = com;

        Status->integral_R = 0.0;
        Status->integral_L = 0.0;
    }

    if (Status->desired_support == 2)
    {
        Eigen::Vector3d com;
        CalcCoM(uLINK, com);

        stab(16) = - 100.0 * (Status->FootCenter_L(0) - com(0)) + 20.0 * (com(0) - Status->com_old(0)) / Dtime;
        stab(17) = + 200.0 * (Status->FootCenter_L(1) - com(1)) - 20.0 * (com(1) - Status->com_old(1)) / Dtime;

        Status->com_old = com;

        Status->integral_R = 0.0;
        Status->integral_L = 0.0;
    }
}
