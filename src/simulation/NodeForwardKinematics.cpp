#include <Eigen/Dense>
#include "uLink.h"
#include "NodeForwardKinematics.h"
#include "Rodrigues.h"

void NodeForwardKinematics(SuLINK uLINK[], int j, int sender)
{
    if (j == 0)
    {
        return;
    }

    if (sender != 0)
    {
        if (uLINK[sender].upper == j)
        {
            if (uLINK[j].child == sender)
            {
                Eigen::Matrix3d rot;
                Rodrigues(rot, uLINK[sender].a, uLINK[sender].q);
                Eigen::Matrix3d rot2 = rot.transpose();
                
                uLINK[j].R = uLINK[sender].R * rot2;
                uLINK[j].p = -uLINK[j].R * uLINK[sender].b + uLINK[sender].p;
            }
            else
            {
                int mom = uLINK[sender].mother;
                Eigen::Matrix3d rot;
                Rodrigues(rot, uLINK[sender].a, uLINK[sender].q);
                Eigen::Matrix3d rot2 = rot.transpose();
                
                uLINK[mom].R = uLINK[sender].R * rot2;
                uLINK[mom].p = -uLINK[mom].R * uLINK[sender].b + uLINK[sender].p;

                NodeForwardKinematics(uLINK, 1, 0);
            }
        }
        else
        {
            int mom = uLINK[j].mother;
            uLINK[j].p = uLINK[mom].R * uLINK[j].b + uLINK[mom].p;
            
            Eigen::Matrix3d rot;
            Rodrigues(rot, uLINK[j].a, uLINK[j].q);
            uLINK[j].R = uLINK[mom].R * rot;
        }
    }

    if (uLINK[j].upper != sender)
    {
        NodeForwardKinematics(uLINK, uLINK[j].upper, j);
    }

    if (uLINK[j].sister != sender)
    {
        NodeForwardKinematics(uLINK, uLINK[j].sister, j);
    }

    if (uLINK[j].child != sender)
    {
        NodeForwardKinematics(uLINK, uLINK[j].child, j);
    }
}
