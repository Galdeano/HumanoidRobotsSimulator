#include <Eigen/Dense>
#include "uLink.h"
#include "Rodrigues.h"
#include "ForwardAllKinematics.h"

void ForwardAllKinematics(SuLINK uLINK[], int j)
{
    if (j == 0)
    {
        return;
    }

    if (j != 1)
    {
        int mom = uLINK[j].mother;

        // Position and orientation
        uLINK[j].p = uLINK[mom].R * uLINK[j].b + uLINK[mom].p;

        Eigen::Matrix3d rot;
        Rodrigues(rot, uLINK[j].a, uLINK[j].q);
        uLINK[j].R = uLINK[mom].R * rot;

        // Spatial velocity axes in world frame
        Eigen::Vector3d hw = uLINK[mom].R * uLINK[j].a;
        Eigen::Vector3d hv = uLINK[j].p.cross(hw);

        uLINK[j].hw = hw;
        uLINK[j].hv = hv;

        // Velocities
        uLINK[j].w  = uLINK[mom].w  + hw * uLINK[j].dq;
        uLINK[j].vo = uLINK[mom].vo + hv * uLINK[j].dq;

        // Spatial accelerations helpers
        Eigen::Vector3d dhw = uLINK[mom].w.cross(hw);
        Eigen::Vector3d dhv = uLINK[mom].w.cross(hv) + uLINK[mom].vo.cross(hw);

        // Accelerations
        uLINK[j].dw  = uLINK[mom].dw  + dhw * uLINK[j].dq + hw * uLINK[j].ddq;
        uLINK[j].dvo = uLINK[mom].dvo + dhv * uLINK[j].dq + hv * uLINK[j].ddq;
    }

    ForwardAllKinematics(uLINK, uLINK[j].sister);
    ForwardAllKinematics(uLINK, uLINK[j].child);
}
