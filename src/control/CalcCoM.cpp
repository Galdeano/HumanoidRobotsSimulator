#include <Eigen/Dense>
#include "uLink.h"
#include "CalcMC.h"
#include "TotalMass.h"
#include "CalcCoM.h"

void CalcCoM(SuLINK uLINK[], Eigen::Vector3d & com)
{
    static double M;
    static int init_tmp = 1;
    if (init_tmp == 1)
    {
        M = TotalMass(uLINK, 1);
        init_tmp = 0;
    }

    com.setZero();
    Eigen::Vector3d MC = Eigen::Vector3d::Zero();
    CalcMC(uLINK, MC, 1);
    com = MC / M;
}
